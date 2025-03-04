#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputOpenDrain, Pull},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
};
use log::info;
use onewire::{ds18b20, DeviceSearch, OneWire, DS18B20};

extern crate alloc;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let cell = STATIC_CELL.init(($val));
        cell
    }};
}

static TEMPERATURE_F: AtomicU32 = AtomicU32::new(0);
static SENSOR_CONNECTED: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn cathode_control_task(
    pwm_channel: &'static channel::Channel<'static, LowSpeed>,
    hv_enable_pin: &'static mut Output<'static>,
) {
    pwm_channel.set_duty(0).unwrap();
    hv_enable_pin.set_low();

    let mut hv_active = false;
    let mut duty = 0;
    loop {
        // check whether HV output should be toggled
        let hv_enable = SENSOR_CONNECTED.load(Ordering::Relaxed) == 1;
        if !hv_active && hv_enable {
            info!("Enabling HV out");
            hv_enable_pin.set_high();
            hv_active = true;
            Timer::after(Duration::from_millis(100)).await;
        } else if hv_active && !hv_enable {
            info!("Disabling HV out");
            duty = 0;
            pwm_channel.set_duty(duty).unwrap();
            hv_enable_pin.set_low();
            hv_active = false;
        }

        // if HV is enabled, adjust the duty cycle based on the temperature reading
        if hv_active {
            const PWM_MAX: u8 = 40;
            //const PWM_INC: u8 = 1;
            let temperature_f = TEMPERATURE_F.load(Ordering::Relaxed) as f32 / 10.0;
            let target_duty = if temperature_f <= 60.0 {
                0
            } else if temperature_f >= 90.0 {
                PWM_MAX
            } else {
                ((temperature_f - 60.0) / 30.0 * PWM_MAX as f32) as u8
            };

            while target_duty != duty {
                if pwm_channel.is_duty_fade_running() {
                    continue;
                }

                // let new_duty = if target_duty > duty {
                //     duty + 1
                // } else if target_duty < duty {
                //     duty - 1
                // } else {
                //     duty
                // };
                let new_duty = target_duty;

                let duty_percent = (duty as f32 / PWM_MAX as f32) * 100.0;
                let new_duty_percent = (new_duty as f32 / PWM_MAX as f32) * 100.0;
                info!(
                    "Updating duty cycle {}%=>{}% (actual), {:.1}%=>{:.1}% (range adjusted)",
                    duty, new_duty, duty_percent, new_duty_percent
                );
                pwm_channel.set_duty(new_duty).unwrap();
                // TODO: this code is super buggy, maybe just called hw directly
                //pwm_channel.start_duty_fade(duty, new_duty, 100).unwrap();
                duty = new_duty;
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn temperature_task(mut onewire_pin: OutputOpenDrain<'static>) {
    let mut onewire = OneWire::new(&mut onewire_pin, false);
    let mut delay = embassy_time::Delay;
    let mut last_reading: u16 = 0;
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
                        let new_reading = match ds18b20.read_temperature(&mut onewire, &mut delay) {
                            Ok(temperature) => temperature,
                            Err(e) => {
                                info!("Error reading temperature: {:?}", e);
                                break;
                            }
                        };

                        // Process and log the temperature if raw reading changes
                        if new_reading != last_reading {
                            let (integer, fraction) = onewire::ds18b20::split_temp(new_reading);
                            let c_reading = (integer as f32) + (fraction as f32) / 10000.0;
                            let f_reading = c_reading * 9.0 / 5.0 + 32.0;

                            info!(
                                "Current temperature: {:.1}°C ({:.1}°F)",
                                c_reading, f_reading
                            );

                            TEMPERATURE_F.store((f_reading * 10.0) as u32, Ordering::Relaxed);
                            SENSOR_CONNECTED.store(1, Ordering::Relaxed);
                            last_reading = new_reading;
                        }

                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
                _ => {
                    info!("Unknown device type");
                }
            }
        } else {
            info!("No sensor found, retrying in 5 seconds...");
            SENSOR_CONNECTED.store(0, Ordering::Relaxed);
            Timer::after(Duration::from_secs(5)).await;
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let hv_enable_pin = peripherals.GPIO0;
    let pwm_pin = peripherals.GPIO1;
    let onewire_pin = peripherals.GPIO2;

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

    let mut ls_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    ls_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: fugit::HertzU32::Hz(24_000),
        })
        .unwrap();
    let hv_enable_pin = Output::new(hv_enable_pin, Level::Low);

    let mut pwm_channel = ledc.channel::<LowSpeed>(channel::Number::Channel0, pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: mk_static!(timer::Timer<LowSpeed>, ls_timer),
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    spawner
        .spawn(cathode_control_task(
            mk_static!(channel::Channel<LowSpeed>, pwm_channel),
            mk_static!(Output, hv_enable_pin),
        ))
        .unwrap();

    let onewire_pin = OutputOpenDrain::new(onewire_pin, Level::High, Pull::Up);
    spawner.spawn(temperature_task(onewire_pin)).unwrap();

    loop {
        info!("Still alive!");
        Timer::after(Duration::from_secs(30)).await;
    }
}
