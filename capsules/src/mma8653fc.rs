use core::cell::Cell;
use kernel::ReturnCode;
use kernel::common::take_cell::TakeCell;
use kernel::hil;
use kernel::hil::gpio;
use kernel::hil::i2c::{Error, I2CClient, I2CDevice};

pub static mut BUF: [u8; 6] = [0; 6];

#[allow(dead_code)]
enum Registers {
    Status = 0x00,
    OutXMsb = 0x01,
    OutXLsb = 0x02,
    OutYMsb = 0x03,
    OutYLsb = 0x04,
    OutZMsb = 0x05,
    OutZLsb = 0x06,
    Sysmod = 0x0b,
    IntSource = 0x0c,
    WhoAmI = 0x0d,
    XyzDataCfg = 0x0e,
    PlStatus = 0x10,
    PlCfg = 0x11,
    PlCount = 0x12,
    PlBfZcomp = 0x13,
    PlThsReg = 0x14,
    FfMtCfg = 0x15,
    FfMtSrc = 0x16,
    FfMtThs = 0x17,
    FfMtCount = 0x18,
    AslpCount = 0x29,
    CtrlReg1 = 0x2a,
    CtrlReg2 = 0x2b,
    CtrlReg3 = 0x2c,
    CtrlReg4 = 0x2d,
    CtrlReg5 = 0x2e,
    OffX = 0x2f,
    OffY = 0x30,
    OffZ = 0x31,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    /// Sensor is in standby mode
    Disabled,

    /// Activate the accelerometer to take a reading
    ReadAccelSetup,

    /// Wait for the acceleration sample to be ready
    ReadAccelWait,

    /// Activate sensor to take readings
    ReadAccelWaiting,

    /// Reading accelerometer data
    ReadAccelReading,

    /// Deactivate sensor
    ReadAccelDeactivating(i16, i16, i16),
}

#[allow(dead_code)]
pub struct Mma8653fc<'a> {
    i2c: &'a I2CDevice,
    interrupt_pin1: &'a gpio::Pin,
    interrupt_pin2: &'a gpio::Pin,
    state: Cell<State>,
    buffer: TakeCell<'static, [u8]>,
    callback: Cell<Option<&'static hil::sensors::NineDofClient>>,
}

impl<'a> Mma8653fc<'a> {
    pub fn new(
        i2c: &'a I2CDevice,
        interrupt_pin1: &'a gpio::Pin,
        interrupt_pin2: &'a gpio::Pin,
        buffer: &'static mut [u8],
    ) -> Mma8653fc<'a> {
        Mma8653fc {
            i2c: i2c,
            interrupt_pin1: interrupt_pin1,
            interrupt_pin2: interrupt_pin2,
            state: Cell::new(State::Disabled),
            buffer: TakeCell::new(buffer),
            callback: Cell::new(None),
        }
    }

    fn start_read_accel(&self) {
        // Need an interrupt pin
        self.interrupt_pin1.make_input();

        self.buffer.take().map(|buf| {
            self.state.set(State::ReadAccelSetup);
            self.i2c.enable();
            // Configure the data ready interrupt.
            buf[0] = Registers::CtrlReg4 as u8;
            buf[1] = 1; // CtrlReg4 data ready interrupt
            buf[2] = 1; // CtrlReg5 drdy on pin 1
            self.i2c.write(buf, 3);
        });
    }
}

impl<'a> gpio::Client for Mma8653fc<'a> {
    fn fired(&self, _c: usize) {
        self.buffer.take().map(|buffer| {
            self.interrupt_pin1.disable_interrupt();
            // When we get this interrupt we can read the sample.
            self.i2c.enable();
            buffer[0] = Registers::OutXMsb as u8;
            self.i2c.write_read(buffer, 1, 6); // read 6 accel registers for xyz
            self.state.set(State::ReadAccelReading);
        });
    }
}

impl<'a> I2CClient for Mma8653fc<'a> {
    fn command_complete(&self, buffer: &'static mut [u8], _error: Error) {
        if _error != Error::CommandComplete {
            debug!("Error {} in {:?}", _error, self.state.get());
        }
        match self.state.get() {
            State::ReadAccelSetup => {
                self.state.set(State::ReadAccelWait);
                // Setup the interrupt so we know when the sample is ready
                self.interrupt_pin1
                    .enable_interrupt(1, gpio::InterruptMode::FallingEdge);
                // Enable the accelerometer.
                buffer[0] = Registers::CtrlReg1 as u8;
                buffer[1] = 1;
                self.i2c.write(buffer, 2);
            }
            State::ReadAccelWait => {
                if self.interrupt_pin1.read() == false {
                    // Sample is already ready.
                    self.state.set(State::ReadAccelReading);
                    self.interrupt_pin1.disable_interrupt();
                    buffer[0] = Registers::OutXMsb as u8;
                    self.i2c.write_read(buffer, 1, 6); // read 6 accel registers for xyz
                } else {
                    // Wait for the interrupt to trigger
                    self.state.set(State::ReadAccelWaiting);
                    self.buffer.replace(buffer);
                    self.i2c.disable();
                }
            }
            State::ReadAccelReading => {
                let x = (((buffer[0] as i16) << 8) | buffer[1] as i16) >> 2;
                let y = (((buffer[2] as i16) << 8) | buffer[3] as i16) >> 2;
                let z = (((buffer[4] as i16) << 8) | buffer[5] as i16) >> 2;

                let x = ((x as isize) * 244) / 1000;
                let y = ((y as isize) * 244) / 1000;
                let z = ((z as isize) * 244) / 1000;

                self.state
                    .set(State::ReadAccelDeactivating(x as i16, y as i16, z as i16));
                // Now put the chip into standby mode.
                buffer[0] = Registers::CtrlReg1 as u8;
                buffer[1] = 0; // Set the active bit to 0.
                self.i2c.write(buffer, 2);
            }
            State::ReadAccelDeactivating(x, y, z) => {
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

impl<'a> hil::sensors::NineDof for Mma8653fc<'a> {
    fn set_client(&self, client: &'static hil::sensors::NineDofClient) {
        self.callback.set(Some(client));
    }

    fn read_accelerometer(&self) -> ReturnCode {
        self.start_read_accel();
        ReturnCode::SUCCESS
    }
}
