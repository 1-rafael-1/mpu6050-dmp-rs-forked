//! MPU6050 Sensor Calibration
//!
//! Calibration process:
//! - Takes multiple readings while sensor is stationary
//! - Calculates offset values to zero out errors
//! - Compensates for gravity's effect on accelerometer
//! - Stores calibration in sensor registers

#[cfg(feature = "mpu9265")]
use crate::mag::Mag;
use crate::{
    accel::{Accel, AccelFullScale},
    gyro::{Gyro, GyroFullScale},
};
use core::fmt::Debug;

/// Number of warmup iterations (when values are discarded)
pub(crate) const WARMUP_ITERATIONS: usize = 30;
/// Number of iterations for average error computation
pub(crate) const ITERATIONS: usize = 200;
/// Delay between measurements
pub(crate) const DELAY_MS: u32 = 2;

/// Maximum allowed deviation from zero after calibration
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CalibrationThreshold {
    value: i16,
}

impl CalibrationThreshold {
    // Reasonable acceleration threshold value at a given scale.
    pub const fn from_accel_scale(scale: AccelFullScale) -> Self {
        Self {
            value: match scale {
                AccelFullScale::G2 => 8,
                AccelFullScale::G4 => 4,
                AccelFullScale::G8 => 2,
                AccelFullScale::G16 => 1,
            },
        }
    }

    // Reasonable gyro threshold value at a given scale.
    pub const fn from_gyro_scale(scale: GyroFullScale) -> Self {
        Self {
            value: match scale {
                _ => 1,
            },
        }
    }

    /// Get the threshold value
    pub fn value(&self) -> i16 {
        self.value
    }

    /// Check if the given value is within the threshold
    pub(crate) fn is_value_within(self, value: i16) -> bool {
        value.abs() <= self.value
    }

    /// Check if the given acceleration vector is within the threshold
    pub fn is_accel_within(self, accel: &Accel) -> bool {
        self.is_value_within(accel.x())
            && self.is_value_within(accel.y())
            && self.is_value_within(accel.z())
    }

    /// Check if the given gyro vector is within the threshold
    pub fn is_gyro_within(self, gyro: &Gyro) -> bool {
        self.is_value_within(gyro.x())
            && self.is_value_within(gyro.y())
            && self.is_value_within(gyro.z())
    }

    /// Check if the given magnetometer vector is within the threshold (MPU9265 only)
    #[cfg(feature = "mpu9265")]
    pub fn is_mag_within(self, mag: &Mag) -> bool {
        self.is_value_within(mag.x())
            && self.is_value_within(mag.y())
            && self.is_value_within(mag.z())
    }

    /// If the current computed mean value is not acceptable, compute the next likely
    /// calibration offset.
    ///
    /// This is technically the single step of a PID controller where we are using only
    /// the `I` part (`D` is not needed because calibration is not time-dependent,
    /// and `P` because noise is mitigated by working on averages).
    pub fn next_offset(self, current_mean: i16, current_offset: i16) -> i16 {
        // In this PID controller the "error" is the observed average (when the calibration
        // is correct the average is espected to be zero, or anyway within the given threshold).
        if self.is_value_within(current_mean) {
            // If we are withing the expected threshold do not change the offset (there's no need!).
            current_offset
        } else {
            // Otherwise adjust the offset.
            //
            // The current measured mean value is the PID error, and the Ki PID factor is -0.1
            // (we are dividing `current_mean` by 10).
            //
            // The `signum` factor is there because we work in the integer domain and if the error
            // is small `current_mean / 10` is zero and the algorithm does not make progress.
            // Adding the `signum` is negligible during normal operation but ensures that the offset
            // keeps changing during the final tuning runs (when the error is already very small).
            current_offset - ((current_mean / 10) + current_mean.signum())
        }
    }
}

/// Reference gravity direction for calibration.
///
/// During calibration, one axis should point straight down:
/// - XN/XP: X-axis pointing down/up
/// - YN/YP: Y-axis pointing down/up
/// - ZN/ZP: Z-axis pointing down/up
/// - Zero: No gravity compensation
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ReferenceGravity {
    Zero,
    XN,
    XP,
    YN,
    YP,
    ZN,
    ZP,
}

impl ReferenceGravity {
    /// Actual `g` value at a given scale
    fn gravity_value(scale: AccelFullScale) -> i16 {
        match scale {
            AccelFullScale::G2 => 16384,
            AccelFullScale::G4 => 8192,
            AccelFullScale::G8 => 4096,
            AccelFullScale::G16 => 2048,
        }
    }

    /// Acceleration vector representing gravity compensation in the given direction
    pub fn gravity_compensation(self, scale: AccelFullScale) -> Accel {
        match self {
            Self::Zero => Accel::new(0, 0, 0),
            Self::XN => Accel::new(-Self::gravity_value(scale), 0, 0),
            Self::XP => Accel::new(Self::gravity_value(scale), 0, 0),
            Self::YN => Accel::new(0, -Self::gravity_value(scale), 0),
            Self::YP => Accel::new(0, Self::gravity_value(scale), 0),
            Self::ZN => Accel::new(0, 0, -Self::gravity_value(scale)),
            Self::ZP => Accel::new(0, 0, Self::gravity_value(scale)),
        }
    }
}

/// Tracks which sensor axes still need calibration.
///
/// Each bit represents one axis:
/// - Bits 0-2: Accelerometer (X,Y,Z)
/// - Bits 3-5: Gyroscope (X,Y,Z)
#[cfg_attr(
    feature = "mpu9265",
    doc = "- Bits 5-7: Magnetometer (X,Y,Z) (MPU9265 only)"
)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CalibrationActions {
    flags: u8,
}

impl CalibrationActions {
    const ACCEL_X: u8 = 1 << 0;
    const ACCEL_Y: u8 = 1 << 1;
    const ACCEL_Z: u8 = 1 << 2;
    const GYRO_X: u8 = 1 << 3;
    const GYRO_Y: u8 = 1 << 4;
    const GYRO_Z: u8 = 1 << 5;
    #[cfg(feature = "mpu9265")]
    const MAG_X: u8 = 1 << 5;
    #[cfg(feature = "mpu9265")]
    const MAG_Y: u8 = 1 << 6;
    #[cfg(feature = "mpu9265")]
    const MAG_Z: u8 = 1 << 7;

    /// Build an empty bit set
    pub fn empty() -> Self {
        Self { flags: 0 }
    }

    /// Build a full bit set
    pub fn all() -> Self {
        Self {
            flags: if cfg!(feature = "mpu9265") {
                0xff // Include magnetometer bits
            } else {
                0x3f // Only accel and gyro bits
            },
        }
    }

    /// Check if we have nothing more to calibrate
    pub fn is_empty(self) -> bool {
        self.flags == 0
    }

    /// Check if acceleration x axis calibration is required
    pub fn accel_x(self) -> bool {
        self.flags & Self::ACCEL_X != 0
    }
    /// Check if acceleration y axis calibration is required
    pub fn accel_y(self) -> bool {
        self.flags & Self::ACCEL_Y != 0
    }
    /// Check if acceleration z axis calibration is required
    pub fn accel_z(self) -> bool {
        self.flags & Self::ACCEL_Z != 0
    }
    /// Check if gyro x axis calibration is required
    pub fn gyro_x(self) -> bool {
        self.flags & Self::GYRO_X != 0
    }
    /// Check if gyro y axis calibration is required
    pub fn gyro_y(self) -> bool {
        self.flags & Self::GYRO_Y != 0
    }
    /// Check if gyro z axis calibration is required
    pub fn gyro_z(self) -> bool {
        self.flags & Self::GYRO_Z != 0
    }

    #[cfg(feature = "mpu9265")]
    /// Check if magnetometer x axis calibration is required
    pub fn mag_x(self) -> bool {
        self.flags & Self::MAG_X != 0
    }

    #[cfg(feature = "mpu9265")]
    /// Check if magnetometer y axis calibration is required
    pub fn mag_y(self) -> bool {
        self.flags & Self::MAG_Y != 0
    }

    #[cfg(feature = "mpu9265")]
    /// Check if magnetometer z axis calibration is required
    pub fn mag_z(self) -> bool {
        self.flags & Self::MAG_Z != 0
    }

    /// Set the given flag
    fn with_flag(self, value: bool, flag: u8) -> Self {
        Self {
            flags: if value {
                self.flags | flag
            } else {
                self.flags & !flag
            },
        }
    }

    /// Set acceleration x flag
    pub fn with_accel_x(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_X)
    }
    /// Set acceleration y flag
    pub fn with_accel_y(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_Y)
    }
    /// Set acceleration z flag
    pub fn with_accel_z(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_Z)
    }
    /// Set gyro x flag
    pub fn with_gyro_x(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_X)
    }
    /// Set gyro y flag
    pub fn with_gyro_y(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_Y)
    }
    /// Set gyro z flag
    pub fn with_gyro_z(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_Z)
    }

    #[cfg(feature = "mpu9265")]
    /// Set magnetometer x flag
    pub fn with_mag_x(self, value: bool) -> Self {
        self.with_flag(value, Self::MAG_X)
    }

    #[cfg(feature = "mpu9265")]
    /// Set magnetometer y flag
    pub fn with_mag_y(self, value: bool) -> Self {
        self.with_flag(value, Self::MAG_Y)
    }

    #[cfg(feature = "mpu9265")]
    /// Set magnetometer z flag
    pub fn with_mag_z(self, value: bool) -> Self {
        self.with_flag(value, Self::MAG_Z)
    }
}

/// Configuration for the calibration process.
///
/// Parameters control:
/// - Sensor ranges (AccelFullScale, GyroFullScale)
/// - Acceptable error thresholds
/// - Number of samples to take
/// - Gravity compensation direction
#[cfg_attr(feature = "mpu9265", doc = "- Magnetometer calibration threshold")]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CalibrationParameters {
    /// Acceleration scale
    pub accel_scale: AccelFullScale,
    /// Acceleration threshold value
    pub accel_threshold: CalibrationThreshold,
    /// Gyro scale
    pub gyro_scale: GyroFullScale,
    /// Gyro threshold value
    pub gyro_threshold: CalibrationThreshold,
    /// Number of warmup iterations when computing mean values
    pub warmup_iterations: usize,
    /// Number of warmup iterations when computing values
    pub iterations: usize,
    /// Reference gravity (will be subtracted from acceleration readings)
    pub gravity: ReferenceGravity,
    /// Magnetometer threshold value (MPU9265 only)
    #[cfg(feature = "mpu9265")]
    pub mag_threshold: CalibrationThreshold,
}

impl CalibrationParameters {
    /// Create calibration parameters given accel and gyro scale and a reference gravity
    /// (sensible defaults are used for all other parameters)
    pub fn new(
        accel_scale: AccelFullScale,
        gyro_scale: GyroFullScale,
        gravity: ReferenceGravity,
    ) -> Self {
        Self {
            accel_scale,
            accel_threshold: CalibrationThreshold::from_accel_scale(accel_scale),
            gyro_scale,
            gyro_threshold: CalibrationThreshold::from_gyro_scale(gyro_scale),
            warmup_iterations: WARMUP_ITERATIONS,
            iterations: ITERATIONS,
            gravity,
            #[cfg(feature = "mpu9265")]
            mag_threshold: CalibrationThreshold { value: 10 }, // Default magnetometer threshold
        }
    }

    /// Change acceleration threshold
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_accel_threshold(self, threshold: i16) -> Self {
        Self {
            accel_threshold: CalibrationThreshold { value: threshold },
            ..self
        }
    }

    /// Change gyro threshold
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_gyro_threshold(self, threshold: i16) -> Self {
        Self {
            gyro_threshold: CalibrationThreshold { value: threshold },
            ..self
        }
    }

    /// Change warmup iterations count
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_warmup_iterations(self, warmup_iterations: usize) -> Self {
        Self {
            warmup_iterations,
            ..self
        }
    }

    /// Change iterations count
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_iterations(self, iterations: usize) -> Self {
        Self { iterations, ..self }
    }

    /// Change magnetometer threshold (MPU9265 only)
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    #[cfg(feature = "mpu9265")]
    pub fn with_mag_threshold(self, threshold: i16) -> Self {
        Self {
            mag_threshold: CalibrationThreshold { value: threshold },
            ..self
        }
    }
}

/// Accumulates sensor readings during calibration.
///
/// Stores:
/// - Running sum of accelerometer readings
/// - Running sum of gyroscope readings
/// - Gravity compensation vector
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct MeanAccumulator {
    pub ax: i32,
    pub ay: i32,
    pub az: i32,
    pub gx: i32,
    pub gy: i32,
    pub gz: i32,
    #[cfg(feature = "mpu9265")]
    pub mx: i32,
    #[cfg(feature = "mpu9265")]
    pub my: i32,
    #[cfg(feature = "mpu9265")]
    pub mz: i32,
    pub gravity_compensation: Accel,
}

impl MeanAccumulator {
    /// Initializes the means with zero values
    /// (and also fixes the reference gravity compensation)
    pub fn new(accel_scale: AccelFullScale, gravity: ReferenceGravity) -> Self {
        Self {
            ax: 0,
            ay: 0,
            az: 0,
            gx: 0,
            gy: 0,
            gz: 0,
            #[cfg(feature = "mpu9265")]
            mx: 0,
            #[cfg(feature = "mpu9265")]
            my: 0,
            #[cfg(feature = "mpu9265")]
            mz: 0,
            gravity_compensation: gravity.gravity_compensation(accel_scale),
        }
    }

    /// Adds a new sample (subtracting the reference gravity)
    #[cfg(not(feature = "mpu9265"))]
    pub fn add(&mut self, accel: &Accel, gyro: &Gyro) {
        self.ax += (accel.x() as i32) - (self.gravity_compensation.x() as i32);
        self.ay += (accel.y() as i32) - (self.gravity_compensation.y() as i32);
        self.az += (accel.z() as i32) - (self.gravity_compensation.z() as i32);
        self.gx += gyro.x() as i32;
        self.gy += gyro.y() as i32;
        self.gz += gyro.z() as i32;
    }

    /// Adds a new sample (subtracting the reference gravity)
    #[cfg(feature = "mpu9265")]
    pub fn add(&mut self, accel: &Accel, gyro: &Gyro, mag: Option<&Mag>) {
        self.ax += (accel.x() as i32) - (self.gravity_compensation.x() as i32);
        self.ay += (accel.y() as i32) - (self.gravity_compensation.y() as i32);
        self.az += (accel.z() as i32) - (self.gravity_compensation.z() as i32);
        self.gx += gyro.x() as i32;
        self.gy += gyro.y() as i32;
        self.gz += gyro.z() as i32;

        // Add magnetometer readings if available
        if let Some(mag) = mag {
            self.mx += mag.x() as i32;
            self.my += mag.y() as i32;
            self.mz += mag.z() as i32;
        }
    }

    /// Compute average values (consumes `self` because the computation is done)
    #[cfg(not(feature = "mpu9265"))]
    pub fn means(mut self) -> (Accel, Gyro) {
        self.ax /= ITERATIONS as i32;
        self.ay /= ITERATIONS as i32;
        self.az /= ITERATIONS as i32;
        self.gx /= ITERATIONS as i32;
        self.gy /= ITERATIONS as i32;
        self.gz /= ITERATIONS as i32;

        (
            Accel::new(self.ax as i16, self.ay as i16, self.az as i16),
            Gyro::new(self.gx as i16, self.gy as i16, self.gz as i16),
        )
    }

    /// Compute average values (consumes `self` because the computation is done)
    #[cfg(feature = "mpu9265")]
    pub fn means(mut self) -> (Accel, Gyro, Option<Mag>) {
        self.ax /= ITERATIONS as i32;
        self.ay /= ITERATIONS as i32;
        self.az /= ITERATIONS as i32;
        self.gx /= ITERATIONS as i32;
        self.gy /= ITERATIONS as i32;
        self.gz /= ITERATIONS as i32;

        // Only include magnetometer means if they were accumulated
        let mag = if self.mx != 0 || self.my != 0 || self.mz != 0 {
            self.mx /= ITERATIONS as i32;
            self.my /= ITERATIONS as i32;
            self.mz /= ITERATIONS as i32;
            Some(Mag::new(self.mx as i16, self.my as i16, self.mz as i16))
        } else {
            None
        };

        (
            Accel::new(self.ax as i16, self.ay as i16, self.az as i16),
            Gyro::new(self.gx as i16, self.gy as i16, self.gz as i16),
            mag,
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        accel::{Accel, AccelFullScale},
        gyro::Gyro,
    };

    use super::{MeanAccumulator, ReferenceGravity};

    #[cfg(not(feature = "mpu9265"))]
    mod base_tests {
        use super::*;

        #[test]
        fn test_mean_accumulator_with_compensation() {
            // Case #1: attempt to subtract with overflow
            {
                let accel = Accel {
                    x: -1180,
                    y: -32768,
                    z: 32767,
                };
                let gyro = Gyro {
                    x: -3,
                    y: -7,
                    z: -10,
                };

                let mut mean_acc = MeanAccumulator::new(AccelFullScale::G2, ReferenceGravity::ZN);
                mean_acc.add(&accel, &gyro);

                // This test verifies that extreme accelerometer values (32767) don't overflow
                // when combined with gravity compensation (-16384). The calculation must be
                // done in i32 to get the correct result of 49151.
                assert_eq!(mean_acc.az, 49151);
            }
        }

        #[test]
        fn test_mean_accumulator_with_compensation_negate_panic() {
            // Case #2: attempt to subtract with overflow
            {
                let mut mean_acc = MeanAccumulator {
                    ax: -700924,
                    ay: -6520832,
                    az: 3260217,
                    gx: -3345,
                    gy: 770,
                    gz: -7648,
                    gravity_compensation: Accel {
                        x: 0,
                        y: 0,
                        z: -16384,
                    },
                };
                let accel = Accel {
                    x: -3536,
                    y: -32768,
                    z: 32767,
                };
                let gyro = Gyro {
                    x: -105,
                    y: 100,
                    z: -36,
                };

                mean_acc.add(&accel, &gyro);
            }
        }
    }

    #[cfg(feature = "mpu9265")]
    mod mpu9265_tests {
        use super::*;

        #[test]
        fn test_mean_accumulator_with_compensation() {
            // Case #1: attempt to subtract with overflow
            {
                let accel = Accel {
                    x: -1180,
                    y: -32768,
                    z: 32767,
                };
                let gyro = Gyro {
                    x: -3,
                    y: -7,
                    z: -10,
                };

                let mut mean_acc = MeanAccumulator::new(AccelFullScale::G2, ReferenceGravity::ZN);
                mean_acc.add(&accel, &gyro, None);

                // This test verifies that extreme accelerometer values (32767) don't overflow
                // when combined with gravity compensation (-16384). The calculation must be
                // done in i32 to get the correct result of 49151.
                assert_eq!(mean_acc.az, 49151);
            }
        }

        #[test]
        fn test_mean_accumulator_with_compensation_negate_panic() {
            // Case #2: attempt to subtract with overflow
            {
                let mut mean_acc = MeanAccumulator {
                    ax: -700924,
                    ay: -6520832,
                    az: 3260217,
                    gx: -3345,
                    gy: 770,
                    gz: -7648,
                    mx: 0,
                    my: 0,
                    mz: 0,
                    gravity_compensation: Accel {
                        x: 0,
                        y: 0,
                        z: -16384,
                    },
                };
                let accel = Accel {
                    x: -3536,
                    y: -32768,
                    z: 32767,
                };
                let gyro = Gyro {
                    x: -105,
                    y: 100,
                    z: -36,
                };

                mean_acc.add(&accel, &gyro, None);
            }
        }
    }
}
