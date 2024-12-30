//! MPU-9265 specific sensor functionality
//!
//! This module provides additional methods for the MPU-9265's magnetometer
//! when the mpu9265 feature is enabled.

use crate::{
    error::Error,
    mag::{Mag, MagBitMode, MagMode, MagRegister, MAG_ADDR},
    registers::Register,
    sensor::Mpu6050,
};
use embedded_hal::i2c::I2c;

impl<I> Mpu6050<I>
where
    I: I2c,
{
    /// Initialize the AK8963 magnetometer
    pub fn init_mag(&mut self) -> Result<(), Error<I>> {
        // Enable I2C master mode
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 5;
        self.write_register(Register::UserCtrl, value)?;

        // Configure I2C master
        self.write_register(Register::I2CMasterCtrl, 0x0D)?; // 400kHz I2C clock

        // Configure magnetometer for continuous measurement mode
        self.write_mag_register(MagRegister::Control1, MagMode::Continuous2 as u8)?;

        // Set 16-bit output
        let mut value = self.read_mag_register(MagRegister::Control1)?;
        value |= (MagBitMode::Bit16 as u8) << 4;
        self.write_mag_register(MagRegister::Control1, value)?;

        Ok(())
    }

    /// Read magnetometer measurements
    pub fn mag(&mut self) -> Result<Mag, Error<I>> {
        let mut data = [0; 6];

        // Wait for data ready
        loop {
            let status = self.read_mag_register(MagRegister::Status1)?;
            if status & 0x01 != 0 {
                break;
            }
        }

        // Read magnetometer data
        self.read_mag_registers(MagRegister::MeasurementXL, &mut data)?;

        // Check for overflow
        let status2 = self.read_mag_register(MagRegister::Status2)?;
        if status2 & 0x08 != 0 {
            return Err(Error::MagOverflow);
        }

        // Convert to 16-bit values
        let x = i16::from_le_bytes([data[0], data[1]]);
        let y = i16::from_le_bytes([data[2], data[3]]);
        let z = i16::from_le_bytes([data[4], data[5]]);

        Ok(Mag::new(x, y, z))
    }

    /// Read from magnetometer register
    fn read_mag_register(&mut self, reg: MagRegister) -> Result<u8, Error<I>> {
        // Set slave 0 to read from magnetometer register
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR | 0x80)?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)?;
        self.write_register(Register::I2CSlave0Ctrl, 0x81)?; // Enable read, 1 byte

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus)?;
            if status & 0x01 == 0 {
                break;
            }
        }

        // Read the data
        self.read_register(Register::I2CSlave0DI)
    }

    /// Write to magnetometer register
    fn write_mag_register(&mut self, reg: MagRegister, value: u8) -> Result<(), Error<I>> {
        // Set slave 0 to write to magnetometer register
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR)?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)?;
        self.write_register(Register::I2CSlave0DO, value)?;
        self.write_register(Register::I2CSlave0Ctrl, 0x81)?; // Enable write, 1 byte

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus)?;
            if status & 0x01 == 0 {
                break;
            }
        }

        Ok(())
    }

    /// Read multiple magnetometer registers
    fn read_mag_registers(&mut self, reg: MagRegister, data: &mut [u8]) -> Result<(), Error<I>> {
        // Set slave 0 to read from magnetometer registers
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR | 0x80)?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)?;
        self.write_register(Register::I2CSlave0Ctrl, 0x80 | data.len() as u8)?;

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus)?;
            if status & 0x01 == 0 {
                break;
            }
        }

        // Read the data
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = self.read_register(Register::I2CSlave0DI)?;
        }

        Ok(())
    }
}
