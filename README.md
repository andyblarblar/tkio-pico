# tkio-pico
tkio is an embedded interface for controlling RC car ESCs and servos from a PC over UART to a microcontroller. This is an implimentation of tkio for RP2040 boards.

## Building
The source code should flash standard pin configuration pico boards out of the box. 

Most dependancies should be resolved by cargo, but a few command line tools need to be `cargo install`ed:
- `elf2uf2-rs` - this allows for flashing the pico

To flash the pico, do not `cargo build`, but instead `cargo run` or `cargo run --release` with the pico connected to the PC in its boot mode. This will build the repo, then copy
the resulting uf2 over to the pico automatically.

## UART interface
Connection settings: 9600_8_N_1

All commands are deliminated by `!`. Do not use newlines, as they will not work. 

All commands will run until another command is sent.

Commands:
- `F xxx`: Drives forward with some percentage of max speed, as an int. For example `F 10!` will drive forward at 10% power.
- `R xxx`: Reverses with some percentage of max speed as an int. This will first stop the car, then reverse.
- `N`: Stops the motor.
- `S xxx`: Turns the steering to some angle in degrees as an int. The angle is the average of the two wheels. Right angles are negitive, and left are positive. For example:
`S -2` will be right 2 degrees, and `S 20` will be left 20 degrees. 

## Electrical setup
- Pin 8: PWM ESC output
- Pin 10: PWM Servo output
- Pin 0,1: default UART pins
- Ground using any ground pin

On most RC cars you are going to need to connect power from the ESC to servo manually by jumping the two red PWM connectors together. Ensure everything shares a common ground,
and then turn on the ESC before the pico to ensure it gets armed properly.

## Car setup
This repo is currently configured to work on a Traxxas Slash, and will likely work on other cars as well. However, it is very possible for this to fail for other ESC's
and the angle be incorrect for the servo on different chassis. The codebase is designed to be modular to solve this issue. To add support for a new car, add a new 
empty struct that impls the `RcCar` marker trait, with the name of your car. If you need a new ESC, create a struct that impls the `ESC` trait, and copy and modify the code from the `XL5`
struct to meet your needs. Finally, create a new servo struct if the servo your using does not currently have a struct. If it does have a struct, then add a new impl 
`Servo<R: RcCar>` for your car struct created earlier. To actually use this new config, change the `XL5` ESC in main, and the servo implimentation as needed. Select the 
car chassis you want the servo to be configured for using the turbofish.
