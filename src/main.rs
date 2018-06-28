#![no_std]

extern crate atsamd21_hal as hal;
extern crate panic_abort;

use hal::hal::blocking::i2c::WriteRead;
use hal::prelude::*;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::atsamd21g18a::{CorePeripherals, Peripherals};
use hal::sercom::*;

mod eight_segment;
use eight_segment::EightSegment;

const MPU9250_ADDRESS: u8 = 0x68;
const WHOAMI_REGISTER: u8 = 117;

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
    let gclk0 = clocks.gclk0().clone();
    let mut i2c = I2CMaster0::new(
        &clocks.sercom0_core(&gclk0).unwrap(),
        400.khz(),
        peripherals.SERCOM0,
        &mut peripherals.PM,
        pins.pa8.into_pad(&mut pins.port),
        pins.pa9.into_pad(&mut pins.port),
    );

    let mut builtin_led = pins.pb8.into_open_drain_output(&mut pins.port);
    let mut seg_a = pins.pa21.into_open_drain_output(&mut pins.port);
    let mut seg_b = pins.pa20.into_open_drain_output(&mut pins.port);
    let mut seg_c = pins.pa11.into_open_drain_output(&mut pins.port);
    let mut seg_d = pins.pb10.into_open_drain_output(&mut pins.port);
    let mut seg_e = pins.pb11.into_open_drain_output(&mut pins.port);
    let mut seg_f = pins.pa16.into_open_drain_output(&mut pins.port);
    let mut seg_g = pins.pa17.into_open_drain_output(&mut pins.port);
    let mut seg_p = pins.pa10.into_open_drain_output(&mut pins.port);
    let mut eight_segment = EightSegment {
        seg_a: &mut seg_a,
        seg_b: &mut seg_b,
        seg_c: &mut seg_c,
        seg_d: &mut seg_d,
        seg_e: &mut seg_e,
        seg_f: &mut seg_f,
        seg_g: &mut seg_g,
        seg_p: &mut seg_p,
    };
    let mut led_state = true;
    let mut delay = Delay::new(core.SYST, &mut clocks);


    let mut data = [0u8];
    i2c.write_read(MPU9250_ADDRESS, &[WHOAMI_REGISTER], &mut data).unwrap();
    let digits = [(data[0] & 0b1111_0000) >> 4, data[0] & 0b0000_1111];

    let mut digit_index = 0usize;

    loop {
        if digit_index == 0 || digit_index == 1 {
            eight_segment.display(digits[digit_index], false);
            digit_index += 1;
        } else if digit_index >= 2 {
            eight_segment.display(100, true);
            digit_index = 0;
        }



//        eight_segment.display(eight_seg_count, false);
        if led_state {
            builtin_led.set_high();
        } else {
            builtin_led.set_low();
        }
        led_state = !led_state;
//        if eight_seg_count < 9 {
//            eight_seg_count += 1;
//        } else {
//            eight_seg_count = 0;
//        }
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
    }
}
