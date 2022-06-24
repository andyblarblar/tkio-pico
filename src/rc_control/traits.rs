//! Abstractions for RC car actuators.

/// Marker trait for configuring structs for use with certain RC models.
pub trait RcCar {}

pub struct TraxxasSlash2wd;
impl RcCar for TraxxasSlash2wd {}

pub struct ExceedShortCourse;
impl RcCar for ExceedShortCourse {}

/// A drive servo.
///
/// This trait is designed to be specialised for each model of Rc Car, to account for differences in
/// mechanics.
pub trait Servo {
    /// Moves the servo such that the 'virtual' Ackermann wheel is at the passed angle.
    ///
    /// Positive angles are to the left, negative is to the right. All angles are in degrees.
    fn set_angle(&mut self, angle: i16);
}

/// An Electronic Speed Controller.
pub trait Esc {
    /// Stops the motors.
    fn set_neutral(&mut self);

    /// Arms the ESC.
    ///
    /// The PWM pin is set to a neutral value after this function exits.
    fn arm_esc(&mut self);

    /// Sets the motors to a percentage of forward power, in a range of \[0,1\].
    fn set_forward(&mut self, percent_power: f32);

    /// Sets the motors to a percentage of backwards power, in a range of \[0,1\].
    ///
    /// This function does not clear the brake lockout, so it may not actually cause the motors to reverse.
    fn set_raw_reverse(&mut self, percent_power: f32);

    /// Sets the motors to a percentage of backwards power, in a range of \[0,1\].
    ///
    /// This function will clear the reverse lockout.
    fn set_reverse(&mut self, percent_power: f32);
}
