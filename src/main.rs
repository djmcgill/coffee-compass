#![no_std]
#![feature(never_type)]

extern crate panic_abort;

use embedded_hal::blocking::i2c::Write as I2CWrite;
use embedded_hal::blocking::i2c::WriteRead as I2CWriteRead;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::OutputPin;
use atsamd21_hal::prelude::*;
use atsamd21_hal::timer::*;
use core::fmt::Write as FmtWrite;

use atsamd21_hal::clock::GenericClockController;
use atsamd21_hal::delay::Delay;
use atsamd21_hal::atsamd21g18a::{CorePeripherals, Peripherals};
use atsamd21_hal::sercom::*;



use core::f32::consts::PI;
use libm::F32Ext;

const MPU9250_ADDRESS: u8 = 0x68;
const MPU_WHOAMI_REGISTER: u8 = 117;
const MAG_ADDRESS: u8 = 0x0C;
const MAG_WHOAMI_REGISTER: u8 = 0x0;

mod stepper;
mod consts;
use crate::consts::*;

fn main() {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
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

    let rx_pin = pins.pb23.into_pull_down_input(&mut pins.port).into_pad(&mut pins.port);
    let tx_pin = pins.pb22.into_push_pull_output(&mut pins.port).into_pad(&mut pins.port);
    let mut serial = UART5::new(
        &clocks.sercom5_core(&gclk0).unwrap(),
        9600.hz(),
        peripherals.SERCOM5,
        &mut core.NVIC,
        &mut peripherals.PM,
        UART5Pinout::Rx3Tx2 {rx: rx_pin, tx: tx_pin});

    let mut motor_step = pins.pa20.into_open_drain_output(&mut pins.port);
    let mut motor_dir = pins.pa21.into_open_drain_output(&mut pins.port);
    motor_dir.set_low();
    motor_step.set_low();

    let mut stepper = stepper::Stepper {
        current_position: 0,
        step: &mut motor_step,
        step_high: false,
        dir: &mut motor_dir,
        dir_high: false,
    };

    let mut count_down: TimerCounter3 = panic!();
    let mut tracker = stepper.track_position(&mut count_down, 0); // FIXME
    let mut builtin_led = pins.pb8.into_open_drain_output(&mut pins.port);
    let mut led_state = true;
    let mut delay = Delay::new(core.SYST, &mut clocks);
    serial.write_str("Initialised\n").unwrap();

    let mut motor_offset = 0;

    serial.write_str("Motor done\n").unwrap();

    let mut data = [0u8];
    i2c.write_read(MPU9250_ADDRESS, &[MPU_WHOAMI_REGISTER], &mut data).unwrap();
    serial.write_fmt(format_args!("MPU WHOAMI: {:X}\n", data[0])).unwrap();

    // Set bypass mode for the magnetometers
    i2c.write(MPU9250_ADDRESS, &[0x37, 0x02]).unwrap();

    let mut data2 = [0u8];
    i2c.write_read(MAG_ADDRESS, &[MAG_WHOAMI_REGISTER], &mut data2).unwrap();
    serial.write_fmt(format_args!("MAG WHOAMI: {:X}\n", data2[0])).unwrap();

    // Request continuous magnetometer measurements in 16 bits
    //  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
    i2c.write(MAG_ADDRESS, &[0x0A, 0x16]).unwrap();
    serial.write_str("starting loop\n").unwrap();

//    let tracker =

    loop {
        // Read register Status 1 and wait for the DRDY: Data Ready
        //   uint8_t ST1;
        //   do
        //   {
        //     I2Cread(MAG_ADDRESS,0x02,1,&ST1);
        //   }
        //   while (!(ST1&0x01));
        let mut loop_data = [0u8];
        i2c.write_read(MAG_ADDRESS, &[0x02], &mut loop_data).unwrap();
        while (loop_data[0] & 0x01) == 0 {
//             TODO: slight delay, or skip measurements?
            i2c.write_read(MAG_ADDRESS, &[0x02], &mut loop_data).unwrap();
        }

        tracker.set_desired_position(100);
        let _ = tracker.poll();


        // Read magnetometer data  
        //   uint8_t Mag[7];  
        //   I2Cread(MAG_ADDRESS,0x03,7,Mag);
        let mut mag_data = [0x8; 7];
        i2c.write_read(MAG_ADDRESS, &[0x03], &mut mag_data).unwrap();
        serial.write_fmt(format_args!("RAW DATA: {:?}\n", mag_data)).unwrap();

        // Create 16 bits values from 8 bits data
        //   int16_t mx=-(Mag[3]<<8 | Mag[2]);
        //   int16_t my=-(Mag[1]<<8 | Mag[0]);
        //   int16_t mz=-(Mag[5]<<8 | Mag[4]);
        let mx: i16 = - ((mag_data[3] as i16) << 8 | mag_data[2] as i16);
        let my: i16 = - ((mag_data[1] as i16) << 8 | mag_data[0] as i16);
        let mz: i16 = - ((mag_data[5] as i16) << 8 | mag_data[4] as i16);

        let x = -((mx+200) as f32);
        let y = -((my-70) as f32);
        let z = (mz-700) as f32;

        // angle from north, clockwise
        let deg = y.atan2(x) * (180.0 / PI) + 180.0;
        serial.write_fmt(format_args!("{}\t{}\t{}\t{}\n", x, y, z, deg)).unwrap();

        let rad_angle = (HOME_X - DEST_X).atan2(HOME_Y - DEST_Y);
        let deg_angle = rad_angle * (180.0 / PI) + 180.0;
        let mut angle = deg - deg_angle;

        while angle < 0.0 {angle += 360.0}
        while angle > 360.0 {angle -= 360.0}
        serial.write_fmt(format_args!("{}\n", angle)).unwrap();

//   Serial.print (mx+200,DEC);
//   Serial.print ("\t");
//   Serial.print (my-70,DEC);
//   Serial.print ("\t");
//   Serial.print (mz-700,DEC);  
//   Serial.print ("\t");
  
//        if digit_index == 0 || digit_index == 1 {
//            eight_s=gment.display(digits[digit_index], false);
//            digit_index += 1;
//        } else if digit_index >= 2 {
//            eight_segment.blank();
//            digit_index = 0;
//        }

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
    }
}
