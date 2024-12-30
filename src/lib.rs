#![no_std]

pub mod accel;
pub mod address;
pub mod calibration;
#[cfg(feature = "async")]
pub mod calibration_async;
pub mod calibration_blocking;
pub mod clock_source;
pub mod config;
mod dmp_firmware;
pub mod error;
#[cfg(feature = "async")]
pub mod error_async;
pub mod euler;
pub mod fifo;
mod firmware_loader;
#[cfg(feature = "async")]
mod firmware_loader_async;
pub mod gravity;
pub mod gyro;
#[cfg(feature = "mpu9265")]
pub mod mag;
#[cfg(feature = "async")]
pub mod motion;
pub mod quaternion;
pub mod registers;
pub mod sensor;
#[cfg(feature = "async")]
pub mod sensor_async;
#[cfg(feature = "mpu9265")]
pub mod sensor_mpu9265;
pub mod temperature;
pub mod yaw_pitch_roll;

// Re-export commonly used types
pub use accel::{Accel, AccelFullScale};
pub use gyro::{Gyro, GyroFullScale};
#[cfg(feature = "mpu9265")]
pub use mag::{Mag, MagBitMode, MagMode};
pub use sensor::Mpu6050;
