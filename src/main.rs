#![no_std]
#![no_main]

mod traxxas_control;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use pimoroni_pico_lipo_16mb as bsp;
use pimoroni_pico_lipo_16mb::hal;
use pimoroni_pico_lipo_16mb::pac;

use crate::hal::pwm::Slices;
use crate::traxxas_control::XL5;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    sio::Sio,
    watchdog::Watchdog,
};
use embedded_time::rate::Extensions;

#[entry]
fn main() -> ! {
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

    let mut led_pin = pins.led.into_push_pull_output();

    // Setup PWM
    let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm4;
    pwm.enable(); //must enable before configuring

    pwm.output_to(pins.gpio8);
    pwm.set_div_int(255u8); //Divide clock to lowest
    pwm.set_top(((125_000_000 / 255) / (50 - 1)) as u16); //Calculate 50hz period

    let mut xl5 = XL5::new(&mut pwm.channel_a, 50.Hz());

    //Arm ESC TODO encasulate this
    xl5.set_reverse(1.0);
    delay.delay_ms(40);

    xl5.set_forward(1.0);
    delay.delay_ms(40);

    xl5.set_neutral();
    delay.delay_ms(500);

    loop {
        led_pin.set_high().unwrap();
        xl5.set_forward(0.15);
        delay.delay_ms(3000);

        // This is all to reverse TODO make a method
        led_pin.set_high().unwrap();
        xl5.set_reverse(0.1);
        delay.delay_ms(20);

        xl5.set_neutral();
        delay.delay_ms(60);

        xl5.set_reverse(0.15);
        delay.delay_ms(3000);
    }
}

// Notes:
// Each cycle is 20ms for 50hz
//
// Arm with forward-back-neutral OR back-forward-neu, where the neu must be at least 500ms and for/rev are 2 cycles
// forward works at any time
// reverse is rev-neu-rev, with 1 cycle for the first rev and 3 for neu
// brake is any-rev?
