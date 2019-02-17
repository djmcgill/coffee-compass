#![no_std]
#![no_main]

use core::f32::consts::PI;
use core::fmt::Write;

use arduino_mkrzero::atsamd21g18a::{CorePeripherals, Peripherals};
use arduino_mkrzero::clock::{GenericClockController, ClockGenId, ClockSource};
use arduino_mkrzero::delay::Delay;
use arduino_mkrzero::prelude::*;
use arduino_mkrzero::sercom::{self, I2CMaster0, PadPin, UART0, UART3, UART5, UART3Pinout, UART5Pinout};
use arduino_mkrzero::entry;
use arduino_mkrzero::gclk::clkctrl::GENR::*;
use arduino_mkrzero::gclk::genctrl::SRCR::*;
use cortex_m_rt::exception;

use arduino_mkrzero::atsamd21g18a::interrupt;

use nb;
//use libm::F32Ext;

//use crate::consts::*;

//const MPU9250_ADDRESS: u8 = 0x68;
//const MPU_WHOAMI_REGISTER: u8 = 117;
//const MAG_ADDRESS: u8 = 0x0C;
//const MAG_WHOAMI_REGISTER: u8 = 0x0;

//mod stepper;
//mod consts;

static mut SERIAL: Option<UART5> = None;
static mut GPS_SERIAL: Option<UART0> = None;

#[interrupt]
fn SERCOM0() {
    unsafe {
        for serial in &mut SERIAL {
            serial.write_str("INTERRUPT START").unwrap();
        }

        let mut buffer: [char; 82] = ['.'; 82];
        let mut ix: usize = 0;

        'inner: loop {
            match (&mut GPS_SERIAL).as_mut().unwrap().read() {
                Result::Ok(byte) => {
                    if ix < 82 {
                        buffer[ix] = byte as char;
                        ix += 1;
                    } else {
                        break 'inner;
                    }
                }

                Result::Err(nb::Error::WouldBlock) => {
                    break 'inner;
                },
                Result::Err(nb::Error::Other(())) => {
                    break 'inner;
                },
            }
        }
        for serial in &mut SERIAL {
            serial.write_str("Done! Buffer is: ").unwrap();
            for buff_char in &buffer[..] {
                serial.write_fmt(format_args!("{}", buff_char)).unwrap();
            }
            serial.write_str("\n").unwrap();
        }
    }
}


#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = arduino_mkrzero::Pins::new(peripherals.PORT);
    let gclk0 = clocks.gclk0().clone();

    let rx_pin = pins.rx.into_pull_down_input(&mut pins.port).into_pad(&mut pins.port);
    let tx_pin = pins.tx.into_push_pull_output(&mut pins.port).into_pad(&mut pins.port);
    let mut serial = UART5::new(
        &clocks.sercom5_core(&gclk0).unwrap(),
        9600.hz(),
        peripherals.SERCOM5,
        &mut core.NVIC,
        &mut peripherals.PM,
        UART5Pinout::Rx3Tx2 {rx: rx_pin, tx: tx_pin},
    );
    unsafe { SERIAL = Some(serial); }

    let mut builtin_led = pins.led_builtin.into_open_drain_output(&mut pins.port);
    let mut led_state = true;
    let mut delay = Delay::new(core.SYST, &mut clocks);

    unsafe {
        for serial in &mut SERIAL {
            serial.write_str("Initialised\n").unwrap();
        }
    }

    let gclk2 = clocks
        .configure_gclk_divider_and_source(GCLK2, 1, DFLL48M, false)
        .unwrap();
    let gps_rx_pin: sercom::Sercom0Pad1 = pins.scl.into_pull_down_input(&mut pins.port).into_pad(&mut pins.port);
    let gps_tx_pin: sercom::Sercom0Pad0 = pins.sda.into_push_pull_output(&mut pins.port).into_pad(&mut pins.port);
    let mut gps_serial = sercom::UART0::new(
        &clocks.sercom0_core(&gclk2).unwrap(),
        9600.hz(),
        peripherals.SERCOM0,
        &mut core.NVIC,
        &mut peripherals.PM,
        sercom::UART0Pinout::Rx1Tx0 {
            rx: gps_rx_pin,
            tx: gps_tx_pin,
        },
    );
    unsafe {
        GPS_SERIAL = Some(gps_serial);
    }

    unsafe {
        for ref mut serial in &mut SERIAL {
            serial.write_str("GPS Initialised\n").unwrap();
            serial.write_str("\n").unwrap();
        }
    }

    loop {
        unsafe {
            for serial in &mut SERIAL {
                serial.write_str("Loop start").unwrap();
            }
        }

        if led_state {
            builtin_led.set_high();
        } else {
            builtin_led.set_low();
        }
        led_state = !led_state;
        delay.delay_ms(200u8);
    }
}
