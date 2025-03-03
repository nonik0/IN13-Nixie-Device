#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, OutputOpenDrain, Pull},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
};
use log::info;
use onewire::{ds18b20, DeviceSearch, OneWire, DS18B20};
use static_cell::StaticCell;

extern crate alloc;

// DS18b20

#[embassy_executor::task]
async fn cathode_control_task(pwm_channel: &'static channel::Channel<'static, LowSpeed>) {
    loop {
        for duty in 0..=100 {
            //info!("Setting duty cycle to {}%", duty);
            pwm_channel.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(10)).await;
        }
        for duty in (0..=100).rev() {
            //info!("Setting duty cycle to {}%", duty);
            pwm_channel.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

#[embassy_executor::task]
async fn temperature_task(mut onewire_pin: OutputOpenDrain<'static>) {
    let mut onewire = OneWire::new(&mut onewire_pin, false);
    let mut delay = embassy_time::Delay;
    loop {
        let mut search = DeviceSearch::new();
        if let Some(device) = match onewire.search_next(&mut search, &mut delay) {
            Ok(device) => device,
            Err(e) => {
                info!("Error searching for device: {:?}", e);
                None
            }
        } {
            match device.address[0] {
                ds18b20::FAMILY_CODE => {
                    let ds18b20 = match DS18B20::new(device) {
                        Ok(sensor) => sensor,
                        Err(e) => {
                            info!("Error initializing DS18B20: {:?}", e);
                            continue;
                        }
                    };
                    loop {
                        let resolution = match ds18b20.measure_temperature(&mut onewire, &mut delay)
                        {
                            Ok(res) => res,
                            Err(e) => {
                                info!("Error measuring temperature: {:?}", e);
                                break;
                            }
                        };
                        Timer::after(Duration::from_millis(resolution.time_ms() as u64)).await;
                        let raw_temperature =
                            match ds18b20.read_temperature(&mut onewire, &mut delay) {
                                Ok(temperature) => temperature,
                                Err(e) => {
                                    info!("Error reading temperature: {:?}", e);
                                    break;
                                }
                            };

                        // Process and log the temperature
                        let (integer, fraction) = onewire::ds18b20::split_temp(raw_temperature);
                        let temperature_c = (integer as f32) + (fraction as f32) / 10000.0;
                        let temperature_f = temperature_c * 9.0 / 5.0 + 32.0;
                        info!(
                            "Current temperature: {:.2}°C ({:.2}°F)",
                            temperature_c, temperature_f
                        );

                        Timer::after(Duration::from_secs(5)).await;
                    }
                }
                _ => {
                    info!("Unknown device type");
                }
            }
        } else {
            info!("No sensor found, retrying in 5 seconds...");
            Timer::after(Duration::from_secs(5)).await;
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    // let _init = esp_wifi::init(
    //     timer1.timer0,
    //     esp_hal::rng::Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    // )
    // .unwrap();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: fugit::HertzU32::Hz(24_000),
        })
        .unwrap();
    static LSTIMER: StaticCell<timer::Timer<LowSpeed>> = StaticCell::new();
    let lstimer = &*LSTIMER.init(lstimer);

    let pwm_pin = peripherals.GPIO2;
    let mut pwm_channel = ledc.channel::<LowSpeed>(channel::Number::Channel0, pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: lstimer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    static PWM_CHANNEL: StaticCell<channel::Channel<LowSpeed>> = StaticCell::new();
    let pwm_channel = &*PWM_CHANNEL.init(pwm_channel);

    spawner.spawn(cathode_control_task(pwm_channel)).unwrap();

    let onewire_pin = OutputOpenDrain::new(peripherals.GPIO1, Level::Low, Pull::Down);
    spawner.spawn(temperature_task(onewire_pin)).unwrap();

    loop {
        info!("Still alive!");
        Timer::after(Duration::from_secs(30)).await;
    }
}
