//! Contains abstractions that allow for control over Traxxas RC car actuators.

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::PwmPin;
use embedded_time::duration::*;
use embedded_time::rate::{Hertz, Rate};

// Notes:
// Each cycle is 20ms for 50hz
//
// It seems that at least 2 cycles should be used or else less accurate timers may mess up. IE one more cycle than needed.
//
// Arm with forward-back-neutral OR back-forward-neu, where the neu must be at least 500ms and for/rev are 2 cycles
// forward works at any time
// reverse is rev-neu-rev, with 1 cycle for the first rev and 3 for neu
// brake is any-rev?

/// Allows for control over a Traxxas XL5 ESC.
pub struct XL5<'a> {
    chan: &'a mut dyn PwmPin<Duty=u16>,
    max_duty: u16,
    period: Milliseconds,
    delay: &'a mut dyn DelayMs<u32>,
}

impl<'a> XL5<'a> {
    /// Creates a new abstraction over the XL5. The passed PWM channel should be
    /// on the pin connected to the control wire on the ESC connector, running at the passed
    /// frequency.
    pub fn new(
        pwm: &'a mut dyn PwmPin<Duty=u16>,
        freq: impl Into<Hertz>,
        delay: &'a mut dyn DelayMs<u32>,
    ) -> Self {
        let freq = freq.into();
        // Valid frequency range for the ESC
        debug_assert!((50u32..=200u32).contains(&freq.0));

        let per: Milliseconds = freq.to_duration().unwrap();

        XL5 {
            max_duty: pwm.get_max_duty(),
            chan: pwm,
            period: per,
            delay,
        }
    }

    /// Stops the motors.
    pub fn set_neutral(&mut self) {
        self.chan
            .set_duty(((1.5 / self.period.0 as f32) * self.max_duty as f32) as u16)
    }

    /// Arms the ESC.
    pub fn arm_esc(&mut self) {
        self.set_raw_reverse(1.0);
        self.delay.delay_ms(self.period.0 * 3);

        self.set_forward(1.0);
        self.delay.delay_ms(self.period.0 * 3);

        self.set_neutral();
        self.delay.delay_ms(600);
    }

    /// Sets the motors to a percentage of forward power, in a range of \[0,1\].
    pub fn set_forward(&mut self, percent_power: f32) {
        debug_assert!((0.0..=1.0).contains(&percent_power));

        let duty = (1.5 + (percent_power * 0.5)) / self.period.0 as f32 * self.max_duty as f32;

        self.chan.set_duty(duty as u16)
    }

    /// Sets the motors to a percentage of backwards power, in a range of \[0,1\].
    ///
    /// This function does not clear the brake lockout, so it may not actually cause the motors to reverse.
    pub fn set_raw_reverse(&mut self, percent_power: f32) {
        debug_assert!((0.0..=1.0).contains(&percent_power));

        let duty = (1.5 - (percent_power * 0.5)) / self.period.0 as f32 * self.max_duty as f32;

        self.chan.set_duty(duty as u16)
    }

    /// Sets the motors to a percentage of backwards power, in a range of \[0,1\].
    ///
    /// This function will clear the reverse lockout, so it will always reverse after a delay of
    /// at least 80ms.
    pub fn set_reverse(&mut self, percent_power: f32) {
        debug_assert!((0.0..=1.0).contains(&percent_power));

        self.set_raw_reverse(0.1);
        self.delay.delay_ms(self.period.0 * 2);

        self.set_neutral(); // This clears the lockout
        self.delay.delay_ms(self.period.0 * 4);

        self.set_raw_reverse(percent_power);
    }
}
