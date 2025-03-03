#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::ledc::{
    channel,
    timer::{self, TimerIFace},
    LSGlobalClkSource, Ledc, LowSpeed,
};
use log::info;

extern crate alloc;

// DS18b20

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

// loop {
//     // Set up a breathing LED: fade from off to on over a second, then
//     // from on back off over the next second.  Then loop.
//     channel0.start_duty_fade(0, 100, 1000)?;
//     while channel0.is_duty_fade_running() {}
//     channel0.start_duty_fade(100, 0, 1000)?;
//     while channel0.is_duty_fade_running() {}
// }

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // let peripherals = esp_hal::init(esp_hal::Config::default());
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(24),
    })?;

    let led = peripherals.GPIO2;
    let mut channel0 = ledc.channel(channel::Number::Channel0, led);
    // channel0.configure(channel::config::Config {
    //     timer: &lstimer0,
    //     duty_pct: 10,
    //     pin_config: channel::config::PinConfig::PushPull,
    // })?;

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
