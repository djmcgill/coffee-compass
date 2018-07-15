#![no_std]

extern crate atsamd21_hal as hal;
extern crate panic_abort;

#[macro_use(block)]
extern crate nb;

use hal::hal::blocking::i2c::WriteRead;
use hal::prelude::*;
use core::fmt::Write;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::atsamd21g18a::{CorePeripherals, Peripherals};
use hal::sercom::*;

mod eight_segment;
use eight_segment::EightSegment;

const MPU9250_ADDRESS: u8 = 0x68;
const WHOAMI_REGISTER: u8 = 117;
const MAG_ADDRESS: u8 = 0x0C;
const MAG_DEVICE_ID: u8 = 0x0;

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
    let mut serial = BlockingWrite(UART5::new(
        &clocks.sercom5_core(&gclk0).unwrap(),
        9600.hz(),
        peripherals.SERCOM5,
        &mut core.NVIC,
        &mut peripherals.PM,
        UART5Pinout::Rx3Tx2 {rx: rx_pin, tx: tx_pin}));

    serial.0.bwrite_all(b"pre-starting\n").unwrap();
    serial.write_str("starting\n").unwrap();
    let mut builtin_led = pins.pb8.into_open_drain_output(&mut pins.port);

    let mut led_state = true;
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut data = [0u8];
    serial.write_str("reading\n").unwrap();
    let res = i2c.write_read(MAG_ADDRESS, &[MAG_DEVICE_ID], &mut data);
    serial.write_fmt(format_args!("WHOAMI {:?}\n", res)).unwrap();
    serial.write_fmt(format_args!("done printing: {:?}\n", data[0])).unwrap();
//    let foo = format_args!("{}", data[0]);
//    serial.bwrite()

    // Set by pass mode for the magnetometers
    //  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
//    i2c.write(MPU9250_ADDRESS, &[0x37, 0x02]).unwrap();

    // Request continuous magnetometer measurements in 16 bits
    //  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
//    i2c.write(MAG_ADDRESS, &[0x0A, 0x16]).unwrap();

    loop {
        // Read register Status 1 and wait for the DRDY: Data Ready
        //   uint8_t ST1;
        //   do
        //   {
        //     I2Cread(MAG_ADDRESS,0x02,1,&ST1);
        //   }
        //   while (!(ST1&0x01));
//        let mut loop_data = [0u8];
//        i2c.write_read(MAG_ADDRESS, &[0x02], &mut loop_data).unwrap();
//        while (loop_data[0] & 0x01) == 0 {
//             TODO: slight delay, or skip measurements?
//            i2c.write_read(MAG_ADDRESS, &[0x02], &mut loop_data).unwrap();
//        }


        // Read magnetometer data  
        //   uint8_t Mag[7];  
        //   I2Cread(MAG_ADDRESS,0x03,7,Mag);
//        let mut mag_data = [0x8; 7];
//        i2c.write_read(MAG_ADDRESS, &[0x03], &mut mag_data).unwrap();

        // Create 16 bits values from 8 bits data
        //   int16_t mx=-(Mag[3]<<8 | Mag[2]);
        //   int16_t my=-(Mag[1]<<8 | Mag[0]);
        //   int16_t mz=-(Mag[5]<<8 | Mag[4]);
//        let mx: i16 = - ((mag_data[3] as i16) << 8 | mag_data[2] as i16);
//        let my: i16 = - ((mag_data[1] as i16) << 8 | mag_data[0] as i16);
//        let mz: i16 = - ((mag_data[5] as i16) << 8 | mag_data[4] as i16);
  
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
//        serial.write(b"foo2\n").unwrap();

//        serial.bwrite_all(b"Hello, world!").unwrap();
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
        delay.delay_ms(200u8);
    }
}

struct BlockingWrite<W>(W) where W: hal::hal::blocking::serial::Write<u8>;
impl<W: hal::hal::blocking::serial::Write<u8>> core::fmt::Write for BlockingWrite<W> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        match self.0.bwrite_all(s.as_bytes()) {
            Ok(()) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}
