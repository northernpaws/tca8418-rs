// Adds the required attribute to use the crate in embedded contexts.
#![cfg_attr(not(feature = "std"), no_std)]

/// Provides Rust mappings for all of the TCA8418 registers.
///
/// These registers can be used directly with register associated
/// methods on the driver to ensure type and register address safety.
pub mod register;

use embedded_hal_async::i2c::SevenBitAddress;
use proc_bitfield::Bitfield;

use crate::register::KeyEventA;

/// The I2C device address used by all TCA8418 devices.
pub const ADDRESS: SevenBitAddress = 0b0110100;

/// Representation of all the pins available on TCA8418
/// for matrix row/column or GPIO usage.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pin {
    /// WQFN pin 1
    Row7,
    /// WQFN pin 2
    Row6,
    /// WQFN pin 3
    Row5,
    /// WQFN pin 4
    Row4,
    /// WQFN pin 5
    Row3,
    /// WQFN pin 6
    Row2,
    /// WQFN pin 7
    Row1,
    /// WQFN pin 8
    Row0,
    /// WQFN pin 9
    Column0,
    /// WQFN pin 10
    Column1,
    /// WQFN pin 11
    Column2,
    /// WQFN pin 12
    Column3,
    /// WQFN pin 13
    Column4,
    /// WQFN pin 14
    Column5,
    /// WQFN pin 15
    Column6,
    /// WQFN pin 16
    Column7,
    /// WQFN pin 17
    Column8,
    /// WQFN pin 18
    Column9,
}

pub struct Tca8418<'a, I2C> {
    /// Handle to the embedded-hal or embedded-hal-async I2C communication interface.
    device: I2C,

    /// Buffer for writing to a register from the I2C device.
    ///
    /// [register_address, register_value]
    write_buf: &'a mut [u8], // [u8; 2]

    /// Buffer for reading performign a write read.
    ///
    /// [register_address]
    write_read_buf: &'a mut [u8], // [u8; 1]

    /// Buffer for reading a byte from the I2C device.
    ///
    /// [register_address]
    read_buf: &'a mut [u8], // [u8; 1]
}

impl<'a, I2C> Tca8418<'a, I2C> {
    /// Constructs a new instance of the TCA8418 driver.
    ///
    /// The constructor takes 3 buffers used for reading
    /// and writing from the I2C device. These buffers are
    /// created external to the driver because some I2C
    /// peripherals require memory for their DMA transfers
    /// in a specific memory region.
    pub fn new(
        device: I2C,
        write_buf: &'a mut [u8],      // [u8; 2]
        write_read_buf: &'a mut [u8], // [u8; 1]
        read_buf: &'a mut [u8],       // [u8; 1],
    ) -> Self {
        Self {
            device,

            write_buf,
            write_read_buf,
            read_buf,
        }
    }

    fn init_config(&self) -> register::Configuration {
        register::Configuration(0)
            .with_ai(false) // disable auto increment for write operations
            .with_gpi_event_mode_configuration(false) // GPI events tracked when keypad locked
            .with_overflow_mode(true) // overflow data shifts with last event pushing first event out
            .with_interrupt_configuration(true) //  processor interrupt is deasserted for 50 μs and reassert with pending interrupts
            .with_overflow_interrupt_enable(true) // assert INT on overflow
            .with_keypad_lock_interrupt_enable(false) // don't assert interrupt after keypad unlock
            .with_gpi_interrupt_enable(false) // assert INT for GPI events // TODO: change
            .with_key_events_interrupt_enable(true) // assert INT for keypad events
    }
}

/// Implements blocked I2C access using traits from the `embedded-hal-async` crate.
impl<'a, I2C> Tca8418<'a, I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    /// Read the value of a register, manually specifying a register address.
    pub fn read_register_raw_blocking(&mut self, address: u8) -> Result<u8, I2C::Error> {
        // Set up to read from the specified register address.
        let write_buf = &mut self.write_read_buf[..1];
        write_buf[0] = address;

        // Write the register address to read, and then read
        // the response byte containing the register's value.
        let read_buf = &mut self.read_buf[..1];
        self.device.write_read(ADDRESS, write_buf, read_buf)?;

        Ok(read_buf[0])
    }

    /// Reads the value of a register.
    ///
    /// Takes one of the register implementations as a type parameter
    /// to statically derive the register address and result type.
    pub fn read_register_blocking<Register: register::Register>(
        &mut self,
    ) -> Result<Register, I2C::Error> {
        // Set up to read from the statically derived register address.
        let write_buf = &mut self.write_read_buf[..1];
        write_buf[0] = Register::ADDRESS;

        // Write the register address to read, and then read
        // the response byte containing the register's value.
        let read_buf = &mut self.read_buf[..1];
        self.device.write_read(ADDRESS, write_buf, read_buf)?;

        // Convert the result byte into the register representation.
        let result = Register::from(read_buf[0]);
        Ok(result)
    }

    /// Writes a new value for a register, manually specifying a register address and value byte.
    pub fn write_register_raw_blocking(
        &mut self,
        address: u8,
        value: u8,
    ) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = address;
        write_buf[1] = value;
        self.device.write(ADDRESS, write_buf)
    }

    /// Writes to a register.
    pub fn write_register_blocking<Register: register::Register>(
        &mut self,
        reg: Register,
    ) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = Register::ADDRESS;
        write_buf[1] = reg.into();
        self.device.write(ADDRESS, write_buf)
    }

    /// Initializes the TAC8418 with common defaults for the configuration register.
    pub fn init_blocking(&mut self) -> Result<(), I2C::Error> {
        self.write_register_blocking(self.init_config())
    }

    /// Reads the head of the FIFO and returns the key event, or `None` if the FIFO was empty.
    ///
    /// The TCA8418's internal FIFO contains a queue of up to 10 key press and release events,
    /// either from the key matrix or from the one or more of the pins configured as GPIO inputs.
    pub fn poll_fifo_blocking(&mut self) -> Result<Option<KeyEventA>, I2C::Error> {
        // The KEY_EVENT_A (0x04) is the head of the FIFO stack.
        //
        // Each read of KEY_EVENT_A will deincrement the key event count
        // by one, and move all the data in the stack down by one.
        let key_event: register::KeyEventA = self.read_register_blocking()?;

        // A 0 in the key event A register indicates that
        // there are no more key events in the FIFO.
        if key_event.0 == 0_u8 {
            return Ok(None);
        }

        Ok(Some(key_event))
    }

    /// Read the status of a GPIO pin from the corrosponding register for the pin.
    pub fn read_gpio_status_blocking(&mut self, pin: Pin) -> Result<bool, I2C::Error> {
        match pin {
            Pin::Row7
            | Pin::Row6
            | Pin::Row5
            | Pin::Row4
            | Pin::Row3
            | Pin::Row2
            | Pin::Row1
            | Pin::Row0 => {
                let register = self.read_register_blocking::<register::GPIODataStatus1>()?;
                match pin {
                    Pin::Row7 => Ok(register.row_7()),
                    Pin::Row6 => Ok(register.row_6()),
                    Pin::Row5 => Ok(register.row_5()),
                    Pin::Row4 => Ok(register.row_4()),
                    Pin::Row3 => Ok(register.row_3()),
                    Pin::Row2 => Ok(register.row_2()),
                    Pin::Row1 => Ok(register.row_1()),
                    Pin::Row0 => Ok(register.row_0()),
                    _ => panic!(),
                }
            }
            Pin::Column7
            | Pin::Column6
            | Pin::Column5
            | Pin::Column4
            | Pin::Column3
            | Pin::Column2
            | Pin::Column1
            | Pin::Column0 => {
                let register = self.read_register_blocking::<register::GPIODataStatus2>()?;
                match pin {
                    Pin::Column7 => Ok(register.column_7()),
                    Pin::Column6 => Ok(register.column_6()),
                    Pin::Column5 => Ok(register.column_5()),
                    Pin::Column4 => Ok(register.column_4()),
                    Pin::Column3 => Ok(register.column_3()),
                    Pin::Column2 => Ok(register.column_2()),
                    Pin::Column1 => Ok(register.column_1()),
                    Pin::Column0 => Ok(register.column_0()),
                    _ => panic!(),
                }
            }
            Pin::Column9 | Pin::Column8 => {
                let register = self.read_register_blocking::<register::GPIODataStatus3>()?;
                match pin {
                    Pin::Column9 => Ok(register.column_9()),
                    Pin::Column8 => Ok(register.column_8()),
                    _ => panic!(),
                }
            }
        }
    }
}

/// Implements asyncronous I2C access using traits from the `embedded-hal-async` crate.
impl<'a, I2C> Tca8418<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    /// Read the value of a register, manually specifying a register address.
    pub async fn read_register_raw(&mut self, address: u8) -> Result<u8, I2C::Error> {
        // Set up to read from the specified register address.
        let write_buf = &mut self.write_read_buf[..1];
        write_buf[0] = address;

        // Write the register address to read, and then read
        // the response byte containing the register's value.
        let read_buf = &mut self.read_buf[..1];
        self.device.write_read(ADDRESS, write_buf, read_buf).await?;

        Ok(read_buf[0])
    }

    /// Reads the value of a register.
    ///
    /// Takes one of the register implementations as a type parameter
    /// to statically derive the register address and result type.
    pub async fn read_register<Register: register::Register>(
        &mut self,
    ) -> Result<Register, I2C::Error> {
        // Set up to read from the statically derived register address.
        let write_buf = &mut self.write_read_buf[..1];
        write_buf[0] = Register::ADDRESS;

        // Write the register address to read, and then read
        // the response byte containing the register's value.
        let read_buf = &mut self.read_buf[..1];
        self.device.write_read(ADDRESS, write_buf, read_buf).await?;

        // Convert the result byte into the register representation.
        let result = Register::from(read_buf[0]);
        Ok(result)
    }

    /// Writes a new value for a register, manually specifying a register address and value byte.
    pub async fn write_register_raw(&mut self, address: u8, value: u8) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = address;
        write_buf[1] = value;
        self.device.write(ADDRESS, write_buf).await
    }

    /// Writes to a register.
    pub async fn write_register<Register: register::Register>(
        &mut self,
        reg: Register,
    ) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = Register::ADDRESS;
        write_buf[1] = reg.into();
        self.device.write(ADDRESS, write_buf).await
    }

    /// Initializes the TAC8418 with common defaults for the configuration register.
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        self.write_register(self.init_config()).await
    }

    /// Reads the head of the FIFO and returns the key event, or `None` if the FIFO was empty.
    ///
    /// The TCA8418's internal FIFO contains a queue of up to 10 key press and release events,
    /// either from the key matrix or from the one or more of the pins configured as GPIO inputs.
    pub async fn poll_fifo(&mut self) -> Result<Option<KeyEventA>, I2C::Error> {
        // The KEY_EVENT_A (0x04) is the head of the FIFO stack.
        //
        // Each read of KEY_EVENT_A will deincrement the key event count
        // by one, and move all the data in the stack down by one.
        let key_event: register::KeyEventA = self.read_register().await?;

        // A 0 in the key event A register indicates that
        // there are no more key events in the FIFO.
        if key_event.0 == 0_u8 {
            return Ok(None);
        }

        Ok(Some(key_event))
    }

    /// Read the status of a GPIO pin from the corrosponding register for the pin.
    pub async fn read_gpio_status(&mut self, pin: Pin) -> Result<bool, I2C::Error> {
        match pin {
            Pin::Row7
            | Pin::Row6
            | Pin::Row5
            | Pin::Row4
            | Pin::Row3
            | Pin::Row2
            | Pin::Row1
            | Pin::Row0 => {
                let register = self.read_register::<register::GPIODataStatus1>().await?;
                match pin {
                    Pin::Row7 => Ok(register.row_7()),
                    Pin::Row6 => Ok(register.row_6()),
                    Pin::Row5 => Ok(register.row_5()),
                    Pin::Row4 => Ok(register.row_4()),
                    Pin::Row3 => Ok(register.row_3()),
                    Pin::Row2 => Ok(register.row_2()),
                    Pin::Row1 => Ok(register.row_1()),
                    Pin::Row0 => Ok(register.row_0()),
                    _ => panic!(),
                }
            }
            Pin::Column7
            | Pin::Column6
            | Pin::Column5
            | Pin::Column4
            | Pin::Column3
            | Pin::Column2
            | Pin::Column1
            | Pin::Column0 => {
                let register = self.read_register::<register::GPIODataStatus2>().await?;
                match pin {
                    Pin::Column7 => Ok(register.column_7()),
                    Pin::Column6 => Ok(register.column_6()),
                    Pin::Column5 => Ok(register.column_5()),
                    Pin::Column4 => Ok(register.column_4()),
                    Pin::Column3 => Ok(register.column_3()),
                    Pin::Column2 => Ok(register.column_2()),
                    Pin::Column1 => Ok(register.column_1()),
                    Pin::Column0 => Ok(register.column_0()),
                    _ => panic!(),
                }
            }
            Pin::Column9 | Pin::Column8 => {
                let register = self.read_register::<register::GPIODataStatus3>().await?;
                match pin {
                    Pin::Column9 => Ok(register.column_9()),
                    Pin::Column8 => Ok(register.column_8()),
                    _ => panic!(),
                }
            }
        }
    }
}
