//! # Jeshture
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

use mpu6050::*;

use no_std_compat::mem;

// The trait used by formatting macros like write! and writeln!
// use core::fmt::Write as FmtWrite;

// The macro for our start-up function
use cortex_m_rt::entry;

// I2C HAL traits & Types.
// use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Serde support
use serde::{Serialize, Deserialize};

// TODO define custom struct for IMU data
#[derive(Serialize, Deserialize)]
struct IMUData {
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
}

//// The linker will place this boot block at the start of our program image. We
//// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Rust and Bust")
        .product("Jeshture")
        .serial_number("128H")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let i2c_pio = i2c_pio::I2C::<_, _, _, _, hal::gpio::FunctionPio0>::new(
        &mut pio,
        pins.gpio20,
        pins.gpio21,
        sm0,
        100_000.Hz(),
        clocks.system_clock.freq(),
    );

    let mut mpu = Mpu6050::new(i2c_pio);
    // mpu.init(&mut delay).ok();
    match mpu.init(&mut delay) {
        Ok(_) => {
            let _ = serial.write(b"MPU6050 init OK");
        }
        Err(_e) => {
            let _ = serial.write(b"MPU6050 init error:");
        }
    }

    let mut led_pin = pins.led.into_push_pull_output();

    let _ = mpu.setup_motion_detection();

    // let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    loop {
        if mpu.get_motion_detected().unwrap() {
            led_pin.set_high().unwrap();
            // let _ = serial.write(b"MOTION\n");
            // println!("r/p: {:?}", acc);
            // let temp_i16 = i16::from_be_bytes(acc.x as u8) >> 5;
            // let temp_f32 = f32::from(acc.x) * 0.125;
            // let _ = writeln!(serial, "Temperature: {:0.2}°C", temp_f32);
            // let _ = serial.write(&acc.x.to_bits().to_be_bytes());
        } else {
            led_pin.set_low().unwrap();
        }

        // get roll and pitch estimate
        // let acc = mpu.get_acc_angles()?;
        // println!("r/p: {:?}", acc);

        // get temp
        // let temp = mpu.get_temp()?;
        // println!("temp: {:?}c", temp);

        // get gyro data, scaled with sensitivity
        // let gyro = mpu.get_gyro()?;
        // println!("gyro: {:?}", gyro);

        // get accelerometer data, scaled with sensitivity
        // let acc = mpu.get_acc()?;
        // println!("acc: {:?}", acc);

        // delay.delay_ms(1000);

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(_count) => {
                    // // Convert to upper case
                    // buf.iter_mut().take(count).for_each(|b| {
                    //     b.make_ascii_uppercase();
                    // });
                    // // Send back to the host
                    // let mut wr_ptr = &buf[..count];
                    // while !wr_ptr.is_empty() {
                    //     let _ = serial.write(wr_ptr).map(|len| {
                    //         wr_ptr = &wr_ptr[len..];
                    //     });
                    // }
                    let acc = mpu.get_acc().unwrap();
                    let data = IMUData {
                        accel_x: acc.x,
                        accel_y: acc.y,
                        accel_z: acc.z,
                    };
                    let serialized = serde_json_core::to_string::<IMUData, { mem::size_of::<IMUData>() * 8 }>(&data).unwrap();
                    let _ = serial.write(serialized.as_bytes());
                    let _ = serial.write(b"\r\n");
                }
            }
        }
    }
}

// End of file
