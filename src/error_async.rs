//! Error types for asynchronous MPU-6050 operations.
//!
//! This module provides error types specific to async I2C communication
//! and device initialization for the MPU-6050 sensor.

use core::fmt::{Debug, Formatter};
use embedded_hal_async::i2c::I2c;

/// Error that occurs during async initialization of the MPU-6050 sensor.
/// Contains both the error and the I2C interface for error recovery.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InitError<I>
where
    I: I2c,
{
    pub i2c: I,
    pub error: Error<I>,
}

impl<I> Debug for InitError<I>
where
    I: I2c,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

/// Error types that can occur during async sensor operations.
///
/// Represents failures in I2C communication and device validation.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<I>
where
    I: I2c,
{
    /// Error occurred during an I2C write operation
    WriteError(I::Error),
    /// Error occurred during an I2C write-read operation
    WriteReadError(I::Error),
    /// Device did not respond as an MPU-6050
    WrongDevice,
    /// Calibration arithmetic overflow occurred
    CalibrationOverflow,
}

impl<I> Debug for Error<I>
where
    I: I2c,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Self::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Self::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Self::WrongDevice => f.write_str("WrongDevice"),
            Self::CalibrationOverflow => f.write_str("CalibrationOverflow"),
        }
    }
}
