#![no_std]
#![no_main]

pub extern crate rp2040_hal as hal;

extern crate cortex_m_rt;
pub use cortex_m_rt::entry;

pub use hal::pac;
hal::bsp_pins!(
    Gpio0 { name: gpio0 },
    Gpio1 { name: gpio1 },
    Gpio2 { name: screen_sda },
    Gpio3 { name: screen_scl },
    Gpio4 { name: gpio4 },
    Gpio5 { name: gpio5 },
    Gpio6 { name: temp_sensor },
    Gpio7 { name: gpio7 },
    Gpio8 { name: gpio8 },
    Gpio9 { name: gpio9 },
    Gpio10 { name: gpio10 },
    Gpio11 { name: gpio11 },
    Gpio12 { name: gpio12 },
    Gpio13 { name: gpio13 },
    Gpio14 { name: gpio14 },
    Gpio15 { name: gpio15 },
    Gpio16 { name: gpio16 },
    Gpio17 { name: gpio17 },
    Gpio18 { name: gpio18 },
    Gpio19 { name: gpio19 },
    Gpio20 { name: gpio20 },
    Gpio21 { name: gpio21 },
    Gpio22 { name: gpio22 },
    Gpio23 { name: b_power_save },
    Gpio24 { name: vbus_detect },
    Gpio25 { name: led },
    Gpio26 { name: gpio26 },
    Gpio27 { name: gpio27 },
    Gpio28 { name: gpio28 },
    Gpio29 {
        name: voltage_monitor
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
