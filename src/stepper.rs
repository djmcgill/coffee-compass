use embedded_hal::digital::OutputPin;
use atsamd21_hal::hal::timer::CountDown;
use atsamd21_hal::hal::prelude::*;
use atsamd21_hal::time::{Hertz, U32Ext};
use nb::*;
use core::ops::{Generator, GeneratorState};

const MOTOR_REVOLUTION: i32 = 513;

pub struct Stepper<'a> {
    current_position: i32,
    step: &'a mut OutputPin,
    dir: &'a mut OutputPin,
}

struct MoveStepsGenerator<'a: 'c, 'b, 'c, B: 'b + CountDown, P: Into<B::Time> + Clone> {
    remaining_steps: u32,
    stepper: &'c mut Stepper<'a>,
    timer: &'b mut B,
    pin_low: bool,
    waiting: bool,
    delay: P,
}

impl<'a, 'b, 'c: 'a, T: CountDown, P: Into<T::Time> + Clone> MoveStepsGenerator<'a, 'b, 'c, T, P> {
    fn poll(&mut self) -> Result<(), !> {
        // Make sure we're done waiting (by yielding)
        if self.waiting {
            match self.timer.wait() {
                Ok(()) => {},
                Err(Error::WouldBlock) => return Err(Error::WouldBlock),
                Err(Error::Other(_)) => unreachable!(), // Countdown::wait has error type `!`
            }
        }
        // If we get here then we're done waiting
        self.waiting = false;

        if self.remaining_steps == 0 {
            Ok(())
        } else {
            if self.pin_low {
                self.stepper.step.set_high();
                self.remaining_steps -= 1;
                self.timer.start(self.delay.clone());
                Err(Error::WouldBlock)
            } else { // self.pin_high
                self.stepper.step.set_low();
                self.timer.start(self.delay.clone());
                Err(Error::WouldBlock)
            }
        }
    }
}

impl<'a> Stepper<'a> {
    pub fn new(step: &'a mut OutputPin, dir: &'a mut OutputPin) -> Self {
        Stepper {
            current_position: 0,
            step,
            dir,
        }
    }

    fn move_steps<'b, 'c, T: CountDown<Time=Hertz>>(&'c mut self, timer: &'b mut T, steps: u32)
        -> MoveStepsGenerator<'a, 'b, 'c, T, Hertz> {
        MoveStepsGenerator {
            stepper: self,
            timer,
            remaining_steps: steps,
            pin_low: false,
            delay: 1000.hz(),
            waiting: false,
        }
    }
}
