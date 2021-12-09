#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use num_format::{Buffer, Locale};
use panic_halt as _;
use rp_temp::hal::{self, pac, prelude::*};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    // Device initialization
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = hal::sio::Sio::new(pac.SIO);

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_temp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp_temp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup the screen
    let screen_sda = pins.screen_sda.into_mode::<hal::gpio::FunctionI2C>();
    let screen_scl = pins.screen_scl.into_mode::<hal::gpio::FunctionI2C>();
    let screen_i2c = hal::I2C::i2c1(
        pac.I2C1,
        screen_sda,
        screen_scl,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    let interface = I2CDisplayInterface::new(screen_i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    let mut led = pins.led.into_push_pull_output();

    let temp_sensor_pin = pins.temp_sensor.into_readable_output();
    let mut temp_sensor = dht11::Dht11::new(temp_sensor_pin);

    let mut buf = Buffer::new();

    loop {
        led.set_high().unwrap();
        delay.delay_ms(500);
        led.set_low().unwrap();
        delay.delay_ms(500);
        led.set_high().unwrap();
        delay.delay_ms(500);
        led.set_low().unwrap();
        delay.delay_ms(500);

        if let Ok(meas) = temp_sensor.perform_measurement(&mut delay) {
            display.clear();

            // Temperature
            buf.write_formatted(&(meas.temperature / 10), &Locale::en);
            Text::with_baseline("Temp: ", Point::zero(), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(buf.as_str(), Point::new(36, 0), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            // Humidity
            buf.write_formatted(&meas.humidity, &Locale::en);
            Text::with_baseline("Humidity: ", Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(buf.as_str(), Point::new(60, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
        }
    }
}
