use embedded_hal::digital::OutputPin;
use embedded_hal::timer::CountDown;
use arduino_mkrzero::time::{Hertz, U32Ext};
use nb::*;

const MOTOR_REVOLUTION: i32 = 513;
const MOTOR_CLOCKWISE_DIR: bool = true;

pub struct Stepper<'a> {
    pub current_position: i32,
    pub step: &'a mut OutputPin,
    pub step_high: bool,
    pub dir: &'a mut OutputPin,
    pub dir_high: bool,
}

pub struct MoveStepsGenerator<'a: 'c, 'b, 'c, B: 'b + CountDown, P: Into<B::Time> + Clone> {
    desired_position: i32,
    stepper: &'c mut Stepper<'a>,
    timer: &'b mut B,
    waiting: bool,
    delay: P,
}

impl<'a, 'b, 'c: 'a, T: CountDown, P: Into<T::Time> + Clone> MoveStepsGenerator<'a, 'b, 'c, T, P> {
    pub fn set_desired_position(&mut self, new_desired_position: i32) {
        debug_assert!(new_desired_position >= 0);
        debug_assert!(new_desired_position < MOTOR_REVOLUTION);
        self.desired_position = new_desired_position;
    }

    pub fn poll(&mut self) -> Result<(), !> {
        // Make sure we're done waiting (by yielding)
        if self.waiting {
            match self.timer.wait() {
                Ok(()) => {},
                Err(Error::WouldBlock) => return Err(Error::WouldBlock),
                Err(Error::Other(_)) => unreachable!(), // Countdown::wait has error type `!`
            }
            // If we get here then we're done waiting
            self.waiting = false;
        }
        let desired_steps = self.desired_position - self.stepper.current_position;
        if desired_steps == 0 { return Ok(()) }

        // We need to take at least 1 step, and are ready to.

        // Set the direction
        self.stepper.ensure_dir((desired_steps > 0) ^ !MOTOR_CLOCKWISE_DIR);

        // Flip the step pin
        self.stepper.toggle_step();
        self.timer.start(self.delay.clone());
        self.waiting = true;
        if self.stepper.step_high { // the motor has moved
            if self.stepper.dir_high {
                self.stepper.increment_position();
            } else {
                self.stepper.decrement_position();
            }
        }
        Err(Error::WouldBlock)
    }
}

impl<'a> Stepper<'a> {
    fn toggle_step(&mut self) {
        if self.step_high {
            self.step.set_low();
        } else {
            self.step.set_high();
        }
        self.step_high = !self.step_high;
    }

    fn ensure_dir(&mut self, dir: bool) {
        if dir && !self.dir_high {
            self.dir.set_high();
            self.dir_high = true;
        } else if !dir && self.dir_high {
            self.dir.set_low();
            self.dir_high = false;
        }
    }

    pub fn new(step: &'a mut OutputPin, step_high: bool, dir: &'a mut OutputPin, dir_high: bool, initial_position: i32) -> Self {
        Stepper {
            current_position: initial_position,
            step,
            step_high,
            dir,
            dir_high,
        }
    }

    pub fn increment_position(&mut self) {
        self.current_position += 1;
        if self.current_position >= MOTOR_REVOLUTION {
            self.current_position -= MOTOR_REVOLUTION;
        }
    }

    pub fn decrement_position(&mut self) {
        self.current_position -= 1;
        if self.current_position < 0 {
            self.current_position += MOTOR_REVOLUTION;
        }
    }

    pub fn track_position<'b, 'c, T: CountDown<Time=Hertz>>(&'c mut self, timer: &'b mut T, desired_position: i32)
        -> MoveStepsGenerator<'a, 'b, 'c, T, Hertz> {
        MoveStepsGenerator {
            desired_position,
            stepper: self,
            timer,
            delay: 1000.hz(),
            waiting: false,
        }
    }
}
