//! Tock kernel for the BBC micro:bit. </br>
//! This is an nRF51822 SoC (a Cortex M0 core with a BLE transceiver) with many
//! exported pins, LEDs, and buttons. </br>
//! Currently the kernel provides application alarms, and GPIO. </br>
//! It will provide a console
//! once the UART is fully implemented and debugged.
//!
//! ### Pin configuration
//!  4 -> P0.00 -> SCL
//!  5 -> P0.01 -> P2
//!  6 -> P0.02 -> P1
//!  7 -> P0.03 -> P0
//!  8 -> P0.04 -> COL1
//!  9 -> P0.05 -> COL2
//! 10 -> P0.06 -> COL3
//! 11 -> P0.07 -> COL4
//! 14 -> P0.08 -> COL5
//! 15 -> P0.09 -> COL6
//! 16 -> P0.10 -> COL7
//! 17 -> P0.11 -> COL8
//! 18 -> P0.12 -> COL9
//! 19 -> P0.13 -> ROW1
//! 20 -> P0.14 -> ROW2
//! 21 -> P0.15 -> ROW3
//! 22 -> P0.16 -> P0.16
//! 25 -> P0.17 -> BTN_A
//! 26 -> P0.18 -> P0.18
//! 27 -> P0.19 -> NRST
//! 28 -> P0.20 -> P0.20
//! 40 -> P0.21 -> MOSI
//! 41 -> P0.22 -> MISO
//! 42 -> P0.23 -> SCK
//! 43 -> P0.24 -> TXD
//! 44 -> P0.25 -> RXD
//! 45 -> P0.26 -> BTN_B
//! 46 -> P0.27 -> ACC INT2
//! 47 -> P0.28 -> ACC INT1
//! 48 -> P0.29 -> MAG_INT1
//!  3 -> P0.30 -> SDA
//!
//! ### Authors
//! * Philip Levis <pal@cs.stanford.edu>
//! * Anderson Lizardo <anderson.lizardo@gmail.com>
//! * Florian Hars <florian@hars.de>
//! * Date: January 05, 2018

#![no_std]
#![no_main]
#![feature(lang_items, compiler_builtins_lib)]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(debug, debug_gpio, static_init)]
extern crate kernel;
extern crate nrf51;
extern crate nrf5x;

use capsules::alarm::AlarmDriver;
use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules::virtual_i2c::{I2CDevice, MuxI2C};
use kernel::{Chip, SysTick};
use kernel::hil;
use kernel::hil::gpio::PinCtl;
use kernel::hil::uart::UART;
use nrf5x::pinmux::Pinmux;
use nrf5x::rtc::{Rtc, RTC};

#[macro_use]
pub mod io;

mod led_matrix;

// The microbit LED matrix pins.
const LED_COL1: usize = 4;
const LED_COL2: usize = 5;
const LED_COL3: usize = 6;
const LED_COL4: usize = 7;
const LED_COL5: usize = 8;
const LED_COL6: usize = 9;
const LED_COL7: usize = 10;
const LED_COL8: usize = 11;
const LED_COL9: usize = 12;
const LED_ROW1: usize = 13;
const LED_ROW2: usize = 14;
const LED_ROW3: usize = 15;
// The micro:bit button pins
const BUTTON_A_PIN: usize = 17;
const BUTTON_B_PIN: usize = 26;

// State for loading and holding applications.

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::FaultResponse = kernel::process::FaultResponse::Panic;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 1;

#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 8192] = [0; 8192];

static mut PROCESSES: [Option<&'static mut kernel::Process<'static>>; NUM_PROCS] = [None];

pub struct Platform {
    ble_radio: &'static capsules::ble_advertising_driver::BLE<
        'static,
        nrf51::radio::Radio,
        VirtualMuxAlarm<'static, Rtc>,
    >,
    button: &'static capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
    console: &'static capsules::console::Console<'static, nrf51::uart::UART>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
    led_matrix: &'static led_matrix::LEDMatrix<
        'static,
        VirtualMuxAlarm<'static, Rtc>,
        nrf5x::gpio::GPIOPin,
    >,
    ninedof: &'static capsules::ninedof::NineDof<'static>,
    temp: &'static capsules::temperature::TemperatureSensor<'static>,
    alarm: &'static AlarmDriver<'static, VirtualMuxAlarm<'static, Rtc>>,
    rng: &'static capsules::rng::SimpleRng<'static, nrf5x::trng::Trng<'static>>,
}

impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::led::DRIVER_NUM => f(Some(self.led_matrix)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temp)),
            capsules::ninedof::DRIVER_NUM => f(Some(self.ninedof)),
            _ => f(None),
        }
    }
}

#[no_mangle]
pub unsafe fn reset_handler() {
    nrf51::init();

    // LEDs
    let led_pins = static_init!(
        [&'static nrf5x::gpio::GPIOPin; 9],
        [
            &nrf5x::gpio::PORT[LED_COL1],
            &nrf5x::gpio::PORT[LED_COL2],
            &nrf5x::gpio::PORT[LED_COL3],
            &nrf5x::gpio::PORT[LED_COL4],
            &nrf5x::gpio::PORT[LED_COL5],
            &nrf5x::gpio::PORT[LED_COL6],
            &nrf5x::gpio::PORT[LED_COL7],
            &nrf5x::gpio::PORT[LED_COL8],
            &nrf5x::gpio::PORT[LED_COL9],
        ]
    );
    let row_pins = static_init!(
        [&'static nrf5x::gpio::GPIOPin; 3],
        [
            &nrf5x::gpio::PORT[LED_ROW1],
            &nrf5x::gpio::PORT[LED_ROW2],
            &nrf5x::gpio::PORT[LED_ROW3],
        ]
    );
    let button_pins = static_init!(
        [(&'static nrf5x::gpio::GPIOPin, capsules::button::GpioMode); 2],
        [
            (
                &nrf5x::gpio::PORT[BUTTON_A_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ),
            (
                &nrf5x::gpio::PORT[BUTTON_B_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ),
        ]
    );
    let button = static_init!(
        capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
        capsules::button::Button::new(button_pins, kernel::Grant::create())
    );
    for &(btn, _) in button_pins.iter() {
        btn.set_input_mode(kernel::hil::gpio::InputMode::PullNone);
        btn.set_client(button);
    }

    let gpio_pins = static_init!(
        [&'static nrf5x::gpio::GPIOPin; 3],
        [
            &nrf5x::gpio::PORT[3], // P0 Edge connector pads
            &nrf5x::gpio::PORT[2], // P1  |
            &nrf5x::gpio::PORT[1]  // P2  V
         /* TODO: rest of the small pads
         &nrf5x::gpio::PORT[4],  //
         &nrf5x::gpio::PORT[5],  //
         &nrf5x::gpio::PORT[6],  // -----
         &nrf5x::gpio::PORT[16], //
         &nrf5x::gpio::PORT[15], //
         &nrf5x::gpio::PORT[14], //
         &nrf5x::gpio::PORT[13], //
         &nrf5x::gpio::PORT[12], //
             */
        ]
    );

    let gpio = static_init!(
        capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }

    nrf51::i2c::TWIM0.configure(Pinmux::new(0), Pinmux::new(30));

    let sensors_i2c = static_init!(MuxI2C<'static>, MuxI2C::new(&nrf51::i2c::TWIM0));
    nrf51::i2c::TWIM0.set_client(sensors_i2c);
    nrf51::i2c::TWIM0.set_speed(nrf51::i2c::Frequency::FREQUENCY::K100);

    // MMA8653FC accelerometer, device address 0x1d
    let mma8653_i2c = static_init!(I2CDevice, I2CDevice::new(sensors_i2c, 0x1d));
    let mma8653 = static_init!(
        capsules::mma8653fc::Mma8653fc<'static>,
        capsules::mma8653fc::Mma8653fc::new(
            mma8653_i2c,
            &nrf5x::gpio::PORT[28],
            &nrf5x::gpio::PORT[27],
            &mut capsules::mma8653fc::BUF
        )
    );
    mma8653_i2c.set_client(mma8653);
    nrf5x::gpio::PORT[27].set_client(mma8653);
    nrf5x::gpio::PORT[28].set_client(mma8653);

    // MAG3110 magnetometer, device address 0x0e
    let mag3110_i2c = static_init!(I2CDevice, I2CDevice::new(sensors_i2c, 0x0e));
    let mag3110 = static_init!(
        capsules::mag3110::Mag3110<'static>,
        capsules::mag3110::Mag3110::new(
            mag3110_i2c,
            &nrf5x::gpio::PORT[29],
            &mut capsules::mag3110::BUF
        )
    );
    mag3110_i2c.set_client(mag3110);
    nrf5x::gpio::PORT[29].set_client(mag3110);

    let ninedof = static_init!(
        capsules::ninedof::NineDof<'static>,
        capsules::ninedof::NineDof::new(mma8653, kernel::Grant::create())
    );
    hil::sensors::NineDof::set_client(mma8653, ninedof);

    nrf51::uart::UART0.configure_no_flow(Pinmux::new(24) /* tx  */, Pinmux::new(25)); /* rx  */
    let console = static_init!(
        capsules::console::Console<nrf51::uart::UART>,
        capsules::console::Console::new(
            &nrf51::uart::UART0,
            115200,
            &mut capsules::console::WRITE_BUF,
            &mut capsules::console::READ_BUF,
            kernel::Grant::create()
        ),
        224 / 8
    );
    UART::set_client(&nrf51::uart::UART0, console);
    console.initialize();

    // Attach the kernel debug interface to this console
    let kc = static_init!(
        capsules::console::App,
        capsules::console::App::default(),
        480 / 8
    );
    kernel::debug::assign_console_driver(Some(console), kc);

    let rtc = &nrf5x::rtc::RTC;
    rtc.start();
    let mux_alarm = static_init!(MuxAlarm<'static, Rtc>, MuxAlarm::new(&RTC), 16);
    rtc.set_client(mux_alarm);

    let virtual_alarm1 = static_init!(
        VirtualMuxAlarm<'static, Rtc>,
        VirtualMuxAlarm::new(mux_alarm),
        24
    );
    let alarm = static_init!(
        AlarmDriver<'static, VirtualMuxAlarm<'static, Rtc>>,
        AlarmDriver::new(virtual_alarm1, kernel::Grant::create()),
        12
    );
    virtual_alarm1.set_client(alarm);

    let ble_radio_virtual_alarm = static_init!(
        VirtualMuxAlarm<'static, Rtc>,
        VirtualMuxAlarm::new(mux_alarm)
    );

    let led_matrix_virtual_alarm = static_init!(
        VirtualMuxAlarm<'static, Rtc>,
        VirtualMuxAlarm::new(mux_alarm)
    );

    let led_matrix = static_init!(
        led_matrix::LEDMatrix<'static, VirtualMuxAlarm<'static, Rtc>, nrf5x::gpio::GPIOPin>,
        led_matrix::LEDMatrix::new(led_matrix_virtual_alarm, led_pins, row_pins)
    );
    led_matrix_virtual_alarm.set_client(led_matrix);

    let temp = static_init!(
        capsules::temperature::TemperatureSensor<'static>,
        capsules::temperature::TemperatureSensor::new(
            &mut nrf5x::temperature::TEMP,
            kernel::Grant::create()
        ),
        96 / 8
    );
    kernel::hil::sensors::TemperatureDriver::set_client(&nrf5x::temperature::TEMP, temp);

    let rng = static_init!(
        capsules::rng::SimpleRng<'static, nrf5x::trng::Trng>,
        capsules::rng::SimpleRng::new(&mut nrf5x::trng::TRNG, kernel::Grant::create()),
        96 / 8
    );
    nrf5x::trng::TRNG.set_client(rng);

    let ble_radio = static_init!(
        capsules::ble_advertising_driver::BLE<
            'static,
            nrf51::radio::Radio,
            VirtualMuxAlarm<'static, Rtc>,
        >,
        capsules::ble_advertising_driver::BLE::new(
            &mut nrf51::radio::RADIO,
            kernel::Grant::create(),
            &mut capsules::ble_advertising_driver::BUF,
            ble_radio_virtual_alarm
        ),
        256 / 8
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_receive_client(
        &nrf51::radio::RADIO,
        ble_radio,
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_transmit_client(
        &nrf51::radio::RADIO,
        ble_radio,
    );
    ble_radio_virtual_alarm.set_client(ble_radio);

    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf51::clock::CLOCK.low_stop();
    nrf51::clock::CLOCK.high_stop();

    nrf51::clock::CLOCK.low_start();
    nrf51::clock::CLOCK.high_set_freq(nrf51::clock::XtalFreq::F16MHz);
    nrf51::clock::CLOCK.high_start(); // switches external oscillator on
    while !nrf51::clock::CLOCK.low_started() {}
    while !nrf51::clock::CLOCK.high_started() {}

    let platform = Platform {
        ble_radio: ble_radio,
        button: button,
        console: console,
        gpio: gpio,
        led_matrix: led_matrix,
        ninedof: ninedof,
        rng: rng,
        alarm: alarm,
        temp: temp,
    };

    rtc.start();

    led_matrix.start();

    let mut chip = nrf51::chip::NRF51::new();
    chip.systick().reset();
    chip.systick().enable(true);

    debug!("Initialization complete. Entering main loop");
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
    }
    kernel::process::load_processes(
        &_sapps as *const u8,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
    );

    kernel::main(
        &platform,
        &mut chip,
        &mut PROCESSES,
        &kernel::ipc::IPC::new(),
    );
}
