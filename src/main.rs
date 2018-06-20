#![no_std]

extern crate atsamd21_hal as hal;
extern crate panic_abort;

use hal::prelude::*;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::atsamd21g18a::{CorePeripherals, Peripherals};

fn main() {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::new(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pins = peripherals.PORT.split();
    let mut builtin_led = pins.pb8.into_open_drain_output(&mut pins.port);
    let mut delay = Delay::new(core.SYST, &mut clocks);
    loop {
        delay.delay_ms(200u8);
        builtin_led.set_high();
        delay.delay_ms(200u8);
        builtin_led.set_low();
    }
}
