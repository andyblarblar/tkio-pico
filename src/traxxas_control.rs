//! Contains abstractions that allow for control over Traxxas RC car actuators.

use embedded_hal::PwmPin;
use embedded_time::duration::*;
use embedded_time::rate::{Hertz, Rate};

/// Allows for control over a Traxxas XL5 ESC.
pub struct XL5<'a> {
    chan: &'a mut dyn PwmPin<Duty = u16>,
    max_duty: u16,
    period: Milliseconds,
}

impl<'a> XL5<'a> {
    /// Creates a new abstraction over the XL5. The passed PWM channel should be
    /// on the pin connected to the control wire on the ESC connector, running at the passed
    /// frequency.
    pub fn new(pwm: &'a mut dyn PwmPin<Duty = u16>, freq: Hertz) -> Self {
        // Valid frequency range for the ESC
        debug_assert!((50u32..=200u32).contains(&freq.0));

        let per: Milliseconds = freq.to_duration().unwrap();

        XL5 {
            max_duty: pwm.get_max_duty(),
            chan: pwm,
            period: per,
        }
    }

    /// Stops the motors.
    pub fn set_neutral(&mut self) {
        self.chan
            .set_duty(((1.5 / self.period.0 as f32) * self.max_duty as f32) as u16)
    }

    /// Arms the ESC.
    pub fn arm_esc(&mut self) {
        self.chan
            .set_duty(((0.6 / self.period.0 as f32) * self.max_duty as f32) as u16)
    }

    /// Sets the motors to a percentage of forward power, in a range of \[0,1\].
    pub fn set_forward(&mut self, percent_power: f32) {
        debug_assert!((0.0..=1.0).contains(&percent_power));

        let duty = (1.5 + (percent_power * 0.5)) / self.period.0 as f32 * self.max_duty as f32;

        self.chan.set_duty(duty as u16)
    }

    /// Sets the motors to a percentage of backwards power, in a range of \[0,1\].
    pub fn set_reverse(&mut self, percent_power: f32) {
        debug_assert!((0.0..=1.0).contains(&percent_power));

        let duty = (1.5 - (percent_power * 0.5)) / self.period.0 as f32 * self.max_duty as f32;

        self.chan.set_duty(duty as u16)
    }
}
