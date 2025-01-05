//! MPU-9265 specific sensor functionality
//!
//! This module provides additional methods for the MPU-9265's magnetometer
//! when the mpu9265 feature is enabled.

use crate::{
    calibration::{CalibrationParameters, ITERATIONS, WARMUP_ITERATIONS},
    error::Error,
    mag::{Mag, MagBitMode, MagMode, MagRegister, MAG_ADDR},
    registers::Register,
    sensor::Mpu6050,
};
use embedded_hal::i2c::I2c;

#[derive(Debug)]
#[cfg(feature = "mpu9265")]
pub(crate) struct MagCalibration {
    pub offsets: (i16, i16, i16),
    pub sensitivity: (f32, f32, f32),
}

#[cfg(feature = "mpu9265")]
impl<I> Mpu6050<I>
where
    I: I2c,
{
    /// Read magnetometer sensitivity adjustment values from Fuse ROM
    fn read_mag_sensitivity(&mut self) -> Result<(u8, u8, u8), Error<I>> {
        // Enter Fuse ROM access mode
        self.write_mag_register(MagRegister::Control1, MagMode::FuseROM as u8)?;

        // Read sensitivity adjustment values
        let asa_x = self.read_mag_register(MagRegister::ASAX)?;
        let asa_y = self.read_mag_register(MagRegister::ASAY)?;
        let asa_z = self.read_mag_register(MagRegister::ASAZ)?;

        Ok((asa_x, asa_y, asa_z))
    }

    /// Initialize the AK8963 magnetometer
    pub fn init_mag(&mut self) -> Result<(), Error<I>> {
        // Enable I2C master mode
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 5;
        self.write_register(Register::UserCtrl, value)?;

        // Configure I2C master
        self.write_register(Register::I2CMasterCtrl, 0x0D)?; // 400kHz I2C clock

        // Read sensitivity adjustment values and store them
        let (asa_x, asa_y, asa_z) = self.read_mag_sensitivity()?;
        let sx = ((asa_x as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
        let sy = ((asa_y as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
        let sz = ((asa_z as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;

        // Configure magnetometer for continuous measurement mode with 16-bit output
        let config = MagMode::Continuous2 as u8 | (MagBitMode::Bit16 as u8) << 4;
        self.write_mag_register(MagRegister::Control1, config)?;

        // Initialize calibration with sensitivity adjustments but no offsets
        self.mag_calibration = Some(MagCalibration {
            offsets: (0, 0, 0),
            sensitivity: (sx, sy, sz),
        });

        Ok(())
    }

    /// Calibrate the magnetometer and store calibration data
    pub fn calibrate_mag(&mut self, params: &CalibrationParameters) -> Result<(), Error<I>> {
        let mut mx_offset = 0i16;
        let mut my_offset = 0i16;
        let mut mz_offset = 0i16;

        // Get current sensitivity values or read new ones if not initialized
        let (sx, sy, sz) = if let Some(cal) = &self.mag_calibration {
            cal.sensitivity
        } else {
            let (asa_x, asa_y, asa_z) = self.read_mag_sensitivity()?;
            let sx = ((asa_x as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
            let sy = ((asa_y as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
            let sz = ((asa_z as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
            (sx, sy, sz)
        };

        // Calibration loop
        loop {
            let mut mx_sum = 0i32;
            let mut my_sum = 0i32;
            let mut mz_sum = 0i32;
            let mut count = 0;

            // Discard initial readings
            for _ in 0..WARMUP_ITERATIONS {
                let _ = self.mag()?;
            }

            // Take multiple readings and accumulate
            for _ in 0..ITERATIONS {
                if let Ok(mag) = self.mag() {
                    // Apply sensitivity adjustments and current offsets
                    let mx = (mag.x() as f32 * sx) as i16 - mx_offset;
                    let my = (mag.y() as f32 * sy) as i16 - my_offset;
                    let mz = (mag.z() as f32 * sz) as i16 - mz_offset;

                    mx_sum += mx as i32;
                    my_sum += my as i32;
                    mz_sum += mz as i32;
                    count += 1;
                }
            }

            // Calculate means
            let mx_mean = (mx_sum / count) as i16;
            let my_mean = (my_sum / count) as i16;
            let mz_mean = (mz_sum / count) as i16;

            // Check if calibration is complete
            if params.mag_threshold.is_value_within(mx_mean)
                && params.mag_threshold.is_value_within(my_mean)
                && params.mag_threshold.is_value_within(mz_mean)
            {
                // Store final calibration values
                self.mag_calibration = Some(MagCalibration {
                    offsets: (mx_offset, my_offset, mz_offset),
                    sensitivity: (sx, sy, sz),
                });
                break;
            }

            // Update offsets using PID-like controller
            mx_offset = params.mag_threshold.next_offset(mx_mean, mx_offset);
            my_offset = params.mag_threshold.next_offset(my_mean, my_offset);
            mz_offset = params.mag_threshold.next_offset(mz_mean, mz_offset);
        }

        Ok(())
    }

    /// Read magnetometer measurements with calibration applied if available
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

        // Apply calibration if available
        if let Some(cal) = &self.mag_calibration {
            let mx = (x as f32 * cal.sensitivity.0) as i16 - cal.offsets.0;
            let my = (y as f32 * cal.sensitivity.1) as i16 - cal.offsets.1;
            let mz = (z as f32 * cal.sensitivity.2) as i16 - cal.offsets.2;
            Ok(Mag::new(mx, my, mz))
        } else {
            Ok(Mag::new(x, y, z))
        }
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
        for (_i, byte) in data.iter_mut().enumerate() {
            *byte = self.read_register(Register::I2CSlave0DI)?;
        }

        Ok(())
    }
}
