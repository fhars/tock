use core::cell::Cell;
use kernel::ReturnCode;
use kernel::common::take_cell::TakeCell;
use kernel::hil;
use kernel::hil::gpio;
use kernel::hil::i2c::{Error, I2CClient, I2CDevice};

pub static mut BUF: [u8; 6] = [0; 6];

#[allow(dead_code)]
enum Registers {
    DrStatus = 0x00,
    OutXMsb = 0x01,
    OutXLsb = 0x02,
    OutYMsb = 0x03,
    OutYLsb = 0x04,
    OutZMsb = 0x05,
    OutZLsb = 0x06,
    WhoAmI = 0x07,
    Sysmod = 0x08,
    OffXMsb = 0x09,
    OffXLsb = 0x0a,
    OffYMsb = 0x0b,
    OffYLsb = 0x0c,
    OffZMsb = 0x0d,
    OffZLsb = 0x0e,
    DieTemp = 0x0f,
    CtrlReg1 = 0x10,
    CtrlReg2 = 0x11,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    /// Sensor is in standby mode
    Disabled,

    /// Wait for reset register being set
    ReadMagSetup,

    /// Wait for the magnetometer sample to be ready
    ReadMagWait,

    /// Activate sensor to take readings
    ReadMagWaiting,

    /// Reading accelerometer data
    ReadMagReading,
}

#[allow(dead_code)]
pub struct Mag3110<'a> {
    i2c: &'a I2CDevice,
    interrupt_pin1: &'a gpio::Pin,
    state: Cell<State>,
    buffer: TakeCell<'static, [u8]>,
    callback: Cell<Option<&'static hil::sensors::NineDofClient>>,
}

impl<'a> Mag3110<'a> {
    pub fn new(
        i2c: &'a I2CDevice,
        interrupt_pin1: &'a gpio::Pin,
        buffer: &'static mut [u8],
    ) -> Mag3110<'a> {
        Mag3110 {
            i2c: i2c,
            interrupt_pin1: interrupt_pin1,
            state: Cell::new(State::Disabled),
            buffer: TakeCell::new(buffer),
            callback: Cell::new(None),
        }
    }

    fn start_read_mag(&self) {
        self.buffer.take().map(|buf| {
            self.i2c.enable();
            self.state.set(State::ReadMagSetup);
            // Configure magnetometer auto reset
            buf[0] = Registers::CtrlReg2 as u8;
            buf[1] = 0x80;
            self.i2c.write(buf, 2);
        });
    }
}

impl<'a> gpio::Client for Mag3110<'a> {
    fn fired(&self, _c: usize) {
        self.buffer.take().map(|buffer| {
            self.interrupt_pin1.disable_interrupt();
            // When we get this interrupt we can read the sample.
            self.i2c.enable();
            buffer[0] = Registers::OutXMsb as u8;
            self.i2c.write_read(buffer, 1, 6); // read 6 mag registers for xyz
            self.state.set(State::ReadMagReading);
        });
    }
}

impl<'a> I2CClient for Mag3110<'a> {
    fn command_complete(&self, buffer: &'static mut [u8], _error: Error) {
        if _error != Error::CommandComplete {
            debug!("Error {} in {:?}", _error, self.state.get());
        }
        match self.state.get() {
            State::ReadMagSetup => {
                self.state.set(State::ReadMagWait);
                // Setup the interrupt so we know when the sample is ready
                self.interrupt_pin1
                    .enable_interrupt(1, gpio::InterruptMode::RisingEdge);
                // Enable the magnetometer for one shot.
                buffer[0] = Registers::CtrlReg1 as u8;
                buffer[1] = 0x1a;
                self.i2c.write(buffer, 2);
            }
            State::ReadMagWait => {
                if self.interrupt_pin1.read() {
                    // Sample is already ready.
                    self.state.set(State::ReadMagReading);
                    self.interrupt_pin1.disable_interrupt();
                    buffer[0] = Registers::OutXMsb as u8;
                    self.i2c.write_read(buffer, 1, 6); // read 6 mag registers for xyz
                } else {
                    // Wait for the interrupt to trigger
                    self.state.set(State::ReadMagWaiting);
                    self.buffer.replace(buffer);
                    self.i2c.disable();
                }
            }
            State::ReadMagReading => {
                let x = ((buffer[0] as i16) << 8) | buffer[1] as i16;
                let y = ((buffer[2] as i16) << 8) | buffer[3] as i16;
                let z = ((buffer[4] as i16) << 8) | buffer[5] as i16;

                let x = (x as isize) / 10;
                let y = (y as isize) / 10;
                let z = (z as isize) / 10;

                self.state.set(State::Disabled);
                self.i2c.disable();
                self.buffer.replace(buffer);
                self.callback.get().map(|cb| {
                    cb.callback(x as usize, y as usize, z as usize);
                });
            }
            _ => {}
        }
    }
}

impl<'a> hil::sensors::NineDof for Mag3110<'a> {
    fn set_client(&self, client: &'static hil::sensors::NineDofClient) {
        self.callback.set(Some(client));
        // Need an interrupt pin
        self.interrupt_pin1.make_input();
    }

    fn read_magnetometer(&self) -> ReturnCode {
        self.start_read_mag();
        ReturnCode::SUCCESS
    }
}
