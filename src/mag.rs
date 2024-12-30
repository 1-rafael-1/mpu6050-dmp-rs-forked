//! Magnetometer support for MPU9265
//!
//! The MPU9265 includes an AK8963 magnetometer that can be accessed through
//! the MPU's I2C master interface. This module provides the necessary types
//! and functions to configure and read data from the magnetometer.

use core::fmt::Debug;

/// Magnetometer measurement data
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Mag {
    x: i16,
    y: i16,
    z: i16,
}

impl Mag {
    /// Create new magnetometer reading from raw values
    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }

    /// Get X-axis reading
    pub fn x(&self) -> i16 {
        self.x
    }

    /// Get Y-axis reading
    pub fn y(&self) -> i16 {
        self.y
    }

    /// Get Z-axis reading
    pub fn z(&self) -> i16 {
        self.z
    }
}

/// Magnetometer operating modes
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum MagMode {
    /// Power-down mode
    PowerDown = 0b0000,
    /// Single measurement mode
    Single = 0b0001,
    /// Continuous measurement mode 1 (8Hz)
    Continuous1 = 0b0010,
    /// Continuous measurement mode 2 (100Hz)
    Continuous2 = 0b0110,
    /// External trigger measurement mode
    ExternalTrigger = 0b0100,
    /// Self-test mode
    SelfTest = 0b1000,
    /// Fuse ROM access mode
    FuseROM = 0b1111,
}

/// Magnetometer output bit resolution
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum MagBitMode {
    /// 14-bit output resolution
    Bit14 = 0,
    /// 16-bit output resolution
    Bit16 = 1,
}

/// AK8963 magnetometer registers
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum MagRegister {
    /// Device ID register (should read 0x48)
    WhoAmI = 0x00,
    /// Information register
    Info = 0x01,
    /// Status 1 register
    Status1 = 0x02,
    /// Measurement data registers
    MeasurementXL = 0x03,
    MeasurementXH = 0x04,
    MeasurementYL = 0x05,
    MeasurementYH = 0x06,
    MeasurementZL = 0x07,
    MeasurementZH = 0x08,
    /// Status 2 register
    Status2 = 0x09,
    /// Control 1 register
    Control1 = 0x0A,
    /// Control 2 register
    Control2 = 0x0B,
    /// Self-test control register
    SelfTest = 0x0C,
    /// I2C disable register
    I2CDisable = 0x0F,
    /// Sensitivity adjustment values
    ASAX = 0x10,
    ASAY = 0x11,
    ASAZ = 0x12,
}

/// AK8963 magnetometer I2C address
pub const MAG_ADDR: u8 = 0x0C;

/// Status register 1 bits
pub mod status1 {
    /// Data ready bit
    pub const DRDY: u8 = 1 << 0;
    /// Data overrun bit
    pub const DOR: u8 = 1 << 1;
}

/// Status register 2 bits
pub mod status2 {
    /// Magnetic sensor overflow bit
    pub const HOFL: u8 = 1 << 3;
    /// Output bit setting (14/16-bit)
    pub const BIT: u8 = 1 << 4;
}

/// Control register 1 bits
pub mod control1 {
    /// Mode bits mask
    pub const MODE_MASK: u8 = 0x0F;
    /// Output bit mode
    pub const BIT: u8 = 1 << 4;
}

/// Control register 2 bits
pub mod control2 {
    /// Soft reset bit
    pub const SRST: u8 = 1 << 0;
}

/// Self-test register bits
pub mod self_test {
    /// Self-test bit
    pub const TEST: u8 = 1 << 0;
}

/// I2C disable register bits
pub mod i2c_disable {
    /// I2C disable bit
    pub const DISABLE: u8 = 1 << 0;
}
