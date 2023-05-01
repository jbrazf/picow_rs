//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use standalone_rp::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use standalone_rp::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use standalone_rp::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use standalone_rp::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::fmt::Write;
use heapless::String;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // ! Setup 
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        standalone_rp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = standalone_rp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    #[cfg(feature = "rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_picof::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut led_pin = pins.gpio0.into_push_pull_output();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let delay = (clocks.system_clock.freq().to_Hz() * 2) as u64;

    let mut lap_start = timer.get_counter().ticks();
    let mut active = false;
    let mut i = 0;
    // Blink the LED at 1 Hz
    loop {
        let current_tick = timer.get_counter().ticks();

        let _ = serial.write(b"Hello World!\r\n");
        
        write_to_serial(&mut serial, current_tick);

        // without this writing does not work...
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            let _ = serial.read(&mut buf);
        }
    }
}

fn write_to_serial(serial_con: &mut SerialPort<hal::usb::UsbBus>, i: u64) {
    let mut text: String<64> = String::new();
    let _ = writeln!(&mut text, "Write to serial: {}", i);
    let _ = serial_con.write(text.as_bytes());
}

// End of file
/*
writeln!(&mut text, "Current timer ticks: {}", timer.get_counter().ticks()).unwrap();
        let _ = serial.write(text.as_bytes());

        if timer.get_counter().ticks() > lap_start + delay {
            lap_start = timer.get_counter().ticks();

            writeln!(&mut text, "Current timer ticks: {}", i).unwrap();
            let _ = serial.write(text.as_bytes());

            match active {
                true => led_pin.set_low().unwrap(),
                false => led_pin.set_high().unwrap(),
            }
            active = !active;
            i += 1;
        }
 */