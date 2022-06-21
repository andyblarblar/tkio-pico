//! rc_control is a no_std crate for interacting with RC car actuators, such as ESCs and servos.
//!
//! Each trait in this crate should be appropriately tested on each car before being included, so
//! there are be minimal generic implementations. This is achieved by use of the `RcCar` marker trait,
//! which is used to add specialisations for each RC car as they are tested. Because of this, you must
//! specify the type of car you are using at compile time as a generic argument, like so:
//!
//! ``` no-test
//! let mut servo = Traxxas2075::<TraxxasSlash2wd>::new(&mut servo_pwm.channel_a);
//! ```
//!
//! For ESCs, remember to arm the unit before sending commands, if applicable.

pub mod traxxas_control;
pub mod traits;
pub use traits::Servo;
pub use traits::Esc;