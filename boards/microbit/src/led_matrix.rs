//! Provides userspace access to LED matrix on the micro:bit.
//!
//! It maps the 5x5 LED matrix to a linear arrangement.
//!
//! The capsule implements the same syscall interface as the standard
//! LED driver and has the actual layout of the LEDs hard coded.
//!
//! Usage
//! -----
//!
//! ```rust
//! let led_pins = static_init!(
//!     [&'static nrf5x::gpio::GPIOPin; 9],
//!     [&nrf5x::gpio::PORT[LED_COL1],
//!      &nrf5x::gpio::PORT[LED_COL2],
//!      &nrf5x::gpio::PORT[LED_COL3],
//!      &nrf5x::gpio::PORT[LED_COL4],
//!      &nrf5x::gpio::PORT[LED_COL5],
//!      &nrf5x::gpio::PORT[LED_COL6],
//!      &nrf5x::gpio::PORT[LED_COL7],
//!      &nrf5x::gpio::PORT[LED_COL8],
//!      &nrf5x::gpio::PORT[LED_COL9],
//!     ]);
//! let row_pins = static_init!(
//!     [&'static nrf5x::gpio::GPIOPin; 3],
//!     [&nrf5x::gpio::PORT[LED_ROW1],
//!      &nrf5x::gpio::PORT[LED_ROW2],
//!      &nrf5x::gpio::PORT[LED_ROW3],
//!     ]);
//!
//! let led_matrix_virtual_alarm = static_init!(
//!     VirtualMuxAlarm<'static, Rtc>,
//!     VirtualMuxAlarm::new(mux_alarm));
//!
//! let led_matrix = static_init!(
//!     led_matrix::LEDMatrix<'static, VirtualMuxAlarm<'static, Rtc>, nrf5x::gpio::GPIOPin >,
//!     led_matrix::LEDMatrix::new(led_matrix_virtual_alarm, led_pins, row_pins));
//! led_matrix_virtual_alarm.set_client(led_matrix);
//! ```
//!
//! Syscall Interface
//! -----------------
//!
//! - Stability: 2 - Stable
//!
//! ### Command
//!
//! All LED operations are synchronous, so this capsule only uses the `command`
//! syscall.
//!
//! #### `command_num`
//!
//! - `0`: Return the number of LEDs on this platform.
//!   - `data`: Unused.
//!   - Return: Number of LEDs.
//! - `1`: Turn the LED on.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `SUCCESS` if the LED index was valid, `EINVAL` otherwise.
//! - `2`: Turn the LED off.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `SUCCESS` if the LED index was valid, `EINVAL` otherwise.
//! - `3`: Toggle the on/off state of the LED.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `SUCCESS` if the LED index was valid, `EINVAL` otherwise.

use core::cell::Cell;
use kernel::{AppId, Driver, ReturnCode};
use kernel::hil;
use kernel::hil::time::{self, Alarm, Frequency};

/// Syscall driver number.
/// pub const DRIVER_NUM: usize = 0x00000002;

/// PIN configuration
const NUM_LEDS: usize = 25;
/// The LEDs form a logical 3x9 matrix
const LED_POS: [usize; 25] = [
    1 << (1 - 1 + 9 * (1 - 1)),
    1 << (4 - 1 + 9 * (2 - 1)),
    1 << (2 - 1 + 9 * (1 - 1)),
    1 << (5 - 1 + 9 * (2 - 1)),
    1 << (3 - 1 + 9 * (1 - 1)),
    1 << (4 - 1 + 9 * (3 - 1)),
    1 << (5 - 1 + 9 * (3 - 1)),
    1 << (6 - 1 + 9 * (3 - 1)),
    1 << (7 - 1 + 9 * (3 - 1)),
    1 << (8 - 1 + 9 * (3 - 1)),
    1 << (2 - 1 + 9 * (2 - 1)),
    1 << (9 - 1 + 9 * (1 - 1)),
    1 << (3 - 1 + 9 * (2 - 1)),
    1 << (9 - 1 + 9 * (3 - 1)),
    1 << (1 - 1 + 9 * (2 - 1)),
    1 << (8 - 1 + 9 * (1 - 1)),
    1 << (7 - 1 + 9 * (1 - 1)),
    1 << (6 - 1 + 9 * (1 - 1)),
    1 << (5 - 1 + 9 * (1 - 1)),
    1 << (4 - 1 + 9 * (1 - 1)),
    1 << (3 - 1 + 9 * (3 - 1)),
    1 << (7 - 1 + 9 * (2 - 1)),
    1 << (1 - 1 + 9 * (3 - 1)),
    1 << (6 - 1 + 9 * (2 - 1)),
    1 << (2 - 1 + 9 * (3 - 1)),
];

/// Holds the arrays of GPIO row and column pins attached to the LEDs and implements a `Driver`
/// interface to control them.
pub struct LEDMatrix<'a, A: Alarm + 'a, G: hil::gpio::Pin + 'a> {
    alarm: &'a A,
    status: Cell<usize>,
    curr_row: Cell<usize>,
    cols: &'a [&'a G; 9],
    rows: &'a [&'a G; 3],
}

impl<'a, A: Alarm + 'a, G: hil::gpio::Pin + hil::gpio::PinCtl> LEDMatrix<'a, A, G> {
    pub fn new(
        alarm: &'a A,
        col_pins: &'a [&'a G; 9],
        row_pins: &'a [&'a G; 3],
    ) -> LEDMatrix<'a, A, G> {
        // Make all pins output and off
        for &pin in col_pins.as_ref().iter() {
            pin.make_output();
            pin.set();
        }
        for &pin in row_pins.as_ref().iter() {
            pin.make_output();
            pin.clear();
        }

        LEDMatrix {
            alarm: alarm,
            status: Cell::new(0),
            curr_row: Cell::new(2),
            cols: col_pins,
            rows: row_pins,
        }
    }

    pub fn start(&self) {
        self.alarm.set_alarm(
            self.alarm
                .now()
                .wrapping_add(<A::Frequency>::frequency() / 100),
        );
    }
}

impl<'a, A: Alarm + 'a, G: hil::gpio::Pin + hil::gpio::PinCtl> time::Client
    for LEDMatrix<'a, A, G>
{
    fn fired(&self) {
        let row = self.curr_row.get();
        self.rows[row].clear();
        let new_row = if row >= 2 { 0 } else { row + 1 };
        for i in 0..9 {
            if self.status.get() & (1 << (i + 9 * new_row)) != 0 {
                self.cols[i].clear();
            } else {
                self.cols[i].set();
            }
        }
        self.curr_row.set(new_row);
        self.rows[new_row].set();
        self.start();
    }
}

impl<'a, A: Alarm + 'a, G: hil::gpio::Pin + hil::gpio::PinCtl> Driver for LEDMatrix<'a, A, G> {
    /// Control the LEDs.
    ///
    /// ### `command_num`
    ///
    /// - `0`: Returns the number of LEDs on the board. This will always be 0 or
    ///        greater, and therefore also allows for checking for this driver.
    /// - `1`: Turn the LED at index specified by `data` on. Returns `EINVAL` if
    ///        the LED index is not valid.
    /// - `2`: Turn the LED at index specified by `data` off. Returns `EINVAL`
    ///        if the LED index is not valid.
    /// - `3`: Toggle the LED at index specified by `data` on or off. Returns
    ///        `EINVAL` if the LED index is not valid.
    fn command(&self, command_num: usize, data: usize, _: usize, _: AppId) -> ReturnCode {
        match command_num {
            // get number of LEDs
            0 => ReturnCode::SuccessWithValue { value: NUM_LEDS },

            // on
            1 => {
                if data >= NUM_LEDS {
                    ReturnCode::EINVAL /* impossible pin */
                } else {
                    self.status.set(self.status.get() | LED_POS[data]);
                    ReturnCode::SUCCESS
                }
            }

            // off
            2 => {
                if data >= NUM_LEDS {
                    ReturnCode::EINVAL /* impossible pin */
                } else {
                    self.status.set(self.status.get() & !LED_POS[data]);
                    ReturnCode::SUCCESS
                }
            }

            // toggle
            3 => {
                if data >= NUM_LEDS {
                    ReturnCode::EINVAL /* impossible pin */
                } else {
                    self.status.set(self.status.get() ^ LED_POS[data]);
                    ReturnCode::SUCCESS
                }
            }

            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }
}
