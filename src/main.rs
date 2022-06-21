#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    sio::Sio,
    watchdog::Watchdog,
};
use cortex_m::asm::{sev, wfe};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_serial_Read;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use pac::interrupt;
use panic_probe as _;
use pimoroni_pico_lipo_16mb as bsp;
use pimoroni_pico_lipo_16mb::hal;
use pimoroni_pico_lipo_16mb::pac;

use crate::commands::Command;
use crate::hal::gpio::bank0::{Gpio0, Gpio1};
use crate::hal::gpio::{FunctionUart, Pin};
use crate::hal::pwm::Slices;
use crate::hal::uart;
use crate::hal::uart::UartPeripheral;
use crate::pac::UART0;
use crate::rc_control::traits::{Esc, Servo, TraxxasSlash2wd};
use crate::rc_control::traxxas_control::{Traxxas2075, XL5};
use crate::uart::Reader;

mod commands;
mod rc_control;

type UartRx = Option<Reader<UART0, (Pin<Gpio0, FunctionUart>, Pin<Gpio1, FunctionUart>)>>;

static mut UART_RX: UartRx = None;

// Lockless queue that transmits commands from IRQ handlers to the main event loop.
static EVENT_QUEUE: heapless::mpmc::Q8<Command> = heapless::mpmc::Q8::new();

#[entry]
fn main() -> ! {
    //Boilerplate
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup UART
    let uart_pins = (
        pins.gpio0.into_mode::<FunctionUart>(),
        pins.gpio1.into_mode::<FunctionUart>(),
    );
    // Need to perform clock init before using UART or it will freeze.
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    // Uart rx is only accessed from int, tx is just used for debugging
    let (mut urt_rx, urt_tx) = uart.split();
    urt_rx.enable_rx_interrupt();
    unsafe {
        UART_RX = Some(urt_rx);
    }

    // LED is high until killed
    let mut led_pin = pins.led.into_push_pull_output();

    // Setup PWM
    let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    const CLK_DIV_VAL: u8 = 255u8;

    let xl5_pwm = &mut pwm_slices.pwm4;
    xl5_pwm.enable(); //must enable before configuring

    xl5_pwm.output_to(pins.gpio8);
    xl5_pwm.set_div_int(CLK_DIV_VAL); //Divide clock to lowest
    xl5_pwm.set_top(calc_pwm_freq(
        clocks.system_clock.freq().integer(),
        CLK_DIV_VAL,
        100,
    ));

    let servo_pwm = &mut pwm_slices.pwm5;
    servo_pwm.enable();

    servo_pwm.output_to(pins.gpio10);
    servo_pwm.set_div_int(CLK_DIV_VAL);
    servo_pwm.set_top(calc_pwm_freq(
        clocks.system_clock.freq().integer(),
        CLK_DIV_VAL,
        50,
    ));

    let mut xl5 = XL5::new(&mut xl5_pwm.channel_a, 100.Hz(), &mut delay);
    let mut servo = Traxxas2075::<TraxxasSlash2wd>::new(&mut servo_pwm.channel_a);

    xl5.arm_esc();

    led_pin.set_high().unwrap();

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ);
    }

    // Event loop
    loop {
        if let Some(event) = EVENT_QUEUE.dequeue() {
            // Dispatch command in a cs to avoid PPM timing from being messed up.
            cortex_m::interrupt::free(|_| match event {
                Command::EscForward(perc) => {
                    led_pin.set_low().unwrap();
                    xl5.set_forward(perc as f32 / 100.0);
                    urt_tx.write_full_blocking(b"Set forward\n");
                    led_pin.set_high().unwrap();
                }
                Command::EscRev(perc) => {
                    led_pin.set_low().unwrap();
                    xl5.set_reverse(perc as f32 / 100.0);
                    urt_tx.write_full_blocking(b"Set reverse\n");
                    led_pin.set_high().unwrap();
                }
                Command::EscNeutral => {
                    led_pin.set_low().unwrap();
                    xl5.set_neutral();
                    urt_tx.write_full_blocking(b"Set neutral\n");
                    led_pin.set_high().unwrap();
                }
                Command::Die => {
                    led_pin.set_low().unwrap();
                    urt_tx.write_full_blocking(b"DIE >:]");
                    panic!("Killed by request")
                }
                Command::Nop => {}
                Command::Servo(angle) => {
                    led_pin.set_low().unwrap();
                    servo.set_angle(angle);
                    urt_tx.write_full_blocking(b"Set servo angle");
                    led_pin.set_high().unwrap();
                }
            })
        } else {
            // Sleep between calls to avoid burning power
            urt_tx.write_full_blocking(b"Sleeping...\n");
            wfe();
        }
    }
}

/// Handles interrupts fired when the PC has sent at least a portion of a command.
#[interrupt]
fn UART0_IRQ() {
    static mut RX: UartRx = None;
    static mut QUEUE: heapless::Vec<u8, 64> = heapless::Vec::new();

    if RX.is_none() {
        unsafe {
            *RX = UART_RX.take();
        }
    }
    let rx = RX.as_mut().unwrap();

    while let Ok(byte) = rx.read() {
        // Read until !
        if byte == b'!' {
            let command = Command::from(QUEUE.as_slice());
            EVENT_QUEUE.enqueue(command).unwrap();

            QUEUE.clear();

            // Trigger arm event to wake main
            sev();
            return;
        } else {
            QUEUE.push(byte).unwrap();
        }
    }
}

/// Calculates the pwm wrap level for some target frequency (in Hz), given processor base clock frequency (in Hz)
/// and the clock divider value.
const fn calc_pwm_freq(clock_freq: u32, divider: u8, target_freq: u32) -> u16 {
    let wrap_value = (clock_freq / divider as u32) / (target_freq - 1);
    wrap_value as u16
}
