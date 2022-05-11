#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use pimoroni_pico_lipo_16mb as bsp;
use pimoroni_pico_lipo_16mb::hal;
use pimoroni_pico_lipo_16mb::pac;

use pac::interrupt;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    sio::Sio,
    watchdog::Watchdog,
};
use cortex_m::interrupt::Mutex;

use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

// USB devices shared between interrupts and the main proc
static USB_DEVICE: Mutex<RefCell<Option<UsbDevice<hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static USB_SERIAL: Mutex<RefCell<Option<SerialPort<hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));

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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    //Initialise the values we share with the int
    cortex_m::interrupt::free(|cs| {
        // Safety: all reads to usb bus are safe as interrupts are not enabled yet.
        unsafe {
            USB_BUS.replace(usb_bus);
        }

        // Set up the USB Communications Class Device driver
        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap_unchecked() });

        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Intelligent Systems Club")
        .product("TinyKart IO Controller")
        .serial_number("42069")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

        //Initialise the values we share with the int
        USB_SERIAL.borrow(cs).replace(Some(serial));
        USB_DEVICE.borrow(cs).replace(Some(usb_dev));
    });

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut serial = USB_SERIAL.borrow(cs).borrow_mut();
            serial.as_mut().unwrap().write(b"High!").unwrap();
        });
        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        cortex_m::interrupt::free(|cs| {
            let mut serial = USB_SERIAL.borrow(cs).borrow_mut();
            serial.as_mut().unwrap().write(b"Low!").unwrap();
        });
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs| {
        // Handle USB request
        let mut usb_dev = USB_DEVICE.borrow(cs).borrow_mut();
        let mut usb_serial = USB_SERIAL.borrow(cs).borrow_mut();

        usb_dev
            .as_mut()
            .unwrap_unchecked()
            .poll(&mut [usb_serial.as_mut().unwrap_unchecked()]);
    });
}
