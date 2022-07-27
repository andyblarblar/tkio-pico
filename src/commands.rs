//! Commands sent to control the Pico.
//!
//! Each command is detailed below, and is deliminated from the prior by a !.

use ascii::AsciiStr;

/// Commands sent to us from the uart connection.
#[derive(Debug, Clone, Copy, PartialOrd, PartialEq)]
pub enum Command {
    /// Drive at percent power
    ///
    /// Sent in form:
    ///
    /// 'F XXX!' where XXX is any number between 0 and 100.
    EscForward(u8),
    /// Reverse at percent power
    ///
    /// Sent in form:
    ///
    /// 'R XXX!' where XXX is any number between 0 and 100.
    EscRev(u8),
    /// Stop motion
    ///
    /// Sent in form:
    ///
    /// 'N!'
    EscNeutral,
    /// Kill controller
    ///
    /// Sent in form:
    ///
    /// 'D!'
    Die,
    /// Move servo to virtual Ackermann wheel angle
    ///
    /// Sent in form:
    ///
    /// 'S XXX!' where XXX is the angle to move to in degrees. Positive angles are left, negative angles are right.
    Servo(i16),
    /// Does nothing.
    Nop
}

impl From<&[u8]> for Command {
    fn from(other: &[u8]) -> Self {

        //These two funs trim whitespace, so we can always start at index 2. This means that the only way to mess up a command
        //is to add whitespace between the letter and numbers (I think, this may be handled by the str->int conversions).
        fn u8_from_ascii(data: &[u8], start: usize, end: usize) -> u8 {
            let asci = AsciiStr::from_ascii(&data[start..end]).unwrap();
            asci.as_str().trim().parse::<u8>().unwrap()
        }
        fn i16_from_ascii(data: &[u8], start: usize, end: usize) -> i16 {
            let asci = AsciiStr::from_ascii(&data[start..end]).unwrap();
            asci.as_str().trim().parse::<i16>().unwrap()
        }

        match other[0] {
            // Move forward
            b'F' => {
                let percent = u8_from_ascii(other, 2, other.len());
                if percent == 0 { // Remove chances for divide by zero down the line
                    Command::EscNeutral
                } else {
                    Command::EscForward(percent)
                }
            }
            // Reverse
            b'R' => {
                let percent = u8_from_ascii(other, 2, other.len());
                if percent == 0 {
                    Command::EscNeutral
                } else {
                    Command::EscRev(percent)
                }
            }
            // Neutral
            b'N' => Command::EscNeutral,
            // Kill pico
            b'D' => Command::Die,
            // Servo move
            b'S' => {
                let angle = i16_from_ascii(other, 2, other.len());
                Command::Servo(angle)
            }
            _ => Command::Nop,
        }
    }
}