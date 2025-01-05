//! MPU6050 Asynchronous Driver Implementation
//!
//! This module provides a non-blocking interface to the InvenSense MPU-6050 6-axis motion tracking device.
//! It implements all sensor functionality using async/await for efficient I/O operations:
//!
//! Core Features:
//! - Non-blocking I2C communication using embedded-hal-async
//! - Full sensor configuration (scales, filters, clock sources)
//! - Motion sensing (acceleration and gyroscope)
//! - Temperature measurement
//! - Digital Motion Processor (DMP) support
//! - FIFO buffer management
//! - Advanced calibration routines
//!
//! This implementation mirrors the blocking version (sensor.rs) but provides
//! asynchronous alternatives for all operations, making it suitable for
//! embedded systems running async executors.

#[cfg(feature = "mpu9265")]
use crate::mag::{Mag, MagBitMode, MagMode, MagRegister, MAG_ADDR};
use crate::temperature::Temperature;
use crate::{
    accel::{Accel, AccelFullScale},
    address::Address,
    calibration::{CalibrationActions, CalibrationParameters, ReferenceGravity},
    calibration_async::{calibrate, calibration_loop, collect_mean_values},
    clock_source::ClockSource,
    config::DigitalLowPassFilter,
    error_async::{Error, InitError},
    fifo::Fifo,
    gyro::{Gyro, GyroFullScale},
    motion::{MotionConfig, MotionDetected},
    registers::Register,
};

use embedded_hal_async::{delay, i2c::I2c};

/// InvenSense MPU-6050 Driver
pub struct Mpu6050<I>
where
    I: I2c,
{
    i2c: I,
    address: u8,
    #[cfg(feature = "mpu9265")]
    mag_calibration: Option<crate::sensor_mpu9265::MagCalibration>,
}

impl<I> Mpu6050<I>
where
    I: I2c,
{
    /// Construct a new i2c driver for the MPU-6050
    pub async fn new(i2c: I, address: Address) -> Result<Self, InitError<I>> {
        let mut sensor = Self {
            i2c,
            address: address.into(),
            #[cfg(feature = "mpu9265")]
            mag_calibration: None,
        };

        if let Err(error) = sensor.disable_sleep().await {
            Err(InitError {
                error,
                i2c: sensor.i2c,
            })
        } else {
            Ok(sensor)
        }
    }

    /// Returns the underlying I2C peripheral, consuming this driver.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Load DMP firmware and perform all appropriate initialization.
    pub async fn initialize_dmp(
        &mut self,
        delay: &mut impl delay::DelayNs,
    ) -> Result<(), Error<I>> {
        self.reset(delay).await?;
        self.disable_sleep().await?;
        self.reset_signal_path(delay).await?;
        self.disable_dmp().await?;
        self.set_clock_source(ClockSource::Xgyro).await?;
        self.disable_interrupts().await?;
        self.set_fifo_enabled(Fifo::all_disabled()).await?;
        self.set_accel_full_scale(AccelFullScale::G2).await?;
        self.set_sample_rate_divider(4).await?;
        self.set_digital_lowpass_filter(DigitalLowPassFilter::Filter1)
            .await?;
        self.load_firmware().await?;
        self.boot_firmware().await?;
        self.set_gyro_full_scale(GyroFullScale::Deg2000).await?;
        self.enable_fifo().await?;
        self.reset_fifo().await?;
        self.disable_dmp().await?;
        self.enable_dmp().await?;
        Ok(())
    }

    pub(crate) async fn read(&mut self, bytes: &[u8], response: &mut [u8]) -> Result<(), Error<I>> {
        self.i2c
            .write_read(self.address, bytes, response)
            .await
            .map_err(|e| Error::WriteReadError(e))
    }

    pub(crate) async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I>> {
        self.i2c
            .write(self.address, bytes)
            .await
            .map_err(|e| Error::WriteError(e))
    }

    pub(crate) async fn read_register(&mut self, reg: Register) -> Result<u8, Error<I>> {
        let mut buf = [0; 1];
        self.read(&[reg as u8], &mut buf).await?;
        Ok(buf[0])
    }

    pub(crate) async fn read_registers<'a>(
        &mut self,
        reg: Register,
        buf: &'a mut [u8],
    ) -> Result<&'a [u8], Error<I>> {
        self.read(&[reg as u8], buf).await?;
        Ok(buf)
    }

    pub(crate) async fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<I>> {
        self.write(&[reg as u8, value]).await
    }

    /// Perform power reset of the MPU
    pub async fn reset(&mut self, clock: &mut impl delay::DelayNs) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1).await?;
        value |= 1 << 7;
        self.write_register(Register::PwrMgmt1, value).await?;
        clock.delay_ms(200).await;
        Ok(())
    }

    /// Perform reset of the signal path
    pub async fn reset_signal_path(
        &mut self,
        clock: &mut impl delay::DelayNs,
    ) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 0;
        self.write_register(Register::UserCtrl, value).await?;
        clock.delay_ms(200).await;
        Ok(())
    }

    /// Pick the clock-source
    pub async fn set_clock_source(&mut self, clock_source: ClockSource) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1).await?;
        value |= clock_source as u8;
        self.write_register(Register::PwrMgmt1, value).await?;
        Ok(())
    }

    pub async fn disable_interrupts(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0x00).await
    }

    /// Enables a FIFO buffer overflow to generate an interrupt
    pub async fn interrupt_fifo_oflow_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b0001_0000).await
    }

    /// Enables any of the i2c master interrupt sources to generate an interrupt
    pub async fn interrupt_i2c_mst_int_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b0000_1000).await
    }

    /// Enables the Data Ready interrupt, which occurs each time a write
    /// operation to all of the sensor registers has been completed
    pub async fn interrupt_data_ready_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b1000_0000).await
    }

    /// Read the interrupt status register and clear it.
    pub async fn interrupt_read_clear(&mut self) -> Result<u8, Error<I>> {
        self.read_register(Register::IntStatus).await
    }

    /// Calibrates the accelerometer and gyroscope to minimize sensor bias and drift.
    ///
    /// This calibration process:
    /// 1. Takes multiple readings while the sensor is stationary
    /// 2. Calculates offset values to compensate for:
    ///    - Accelerometer bias (deviation from expected gravity vector)
    ///    - Gyroscope drift (non-zero readings when stationary)
    /// 3. Stores these offsets in the sensor's registers
    ///
    /// The calibration should be performed:
    /// - Once during initial setup
    /// - With the sensor perfectly still and properly oriented
    /// - According to the ReferenceGravity parameter (e.g., ZN means Z-axis pointing down)
    ///
    /// The resulting calibration values should be:
    /// - Saved and reused via `set_accel_calibration` and `set_gyro_calibration`
    /// - Stored as constants or in persistent storage
    ///
    /// Note: If the sensor is not properly oriented or moving during calibration,
    /// this function may never complete as it tries to achieve unreachable targets.
    #[cfg(not(feature = "mpu9265"))]
    pub async fn calibrate(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
    ) -> Result<(Accel, Gyro), Error<I>> {
        calibrate(self, delay, parameters).await
    }

    #[cfg(feature = "mpu9265")]
    pub async fn calibrate(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
    ) -> Result<(Accel, Gyro, Option<Mag>), Error<I>> {
        calibrate(self, delay, parameters).await
    }

    /// A building block for performing calibration: collect several samples return their average
    #[cfg(not(feature = "mpu9265"))]
    pub async fn collect_mean_values(
        &mut self,
        delay: &mut impl delay::DelayNs,
        accel_scale: AccelFullScale,
        gravity: ReferenceGravity,
    ) -> Result<(Accel, Gyro), Error<I>> {
        collect_mean_values(self, delay, accel_scale, gravity).await
    }

    #[cfg(feature = "mpu9265")]
    pub async fn collect_mean_values(
        &mut self,
        delay: &mut impl delay::DelayNs,
        accel_scale: AccelFullScale,
        gravity: ReferenceGravity,
    ) -> Result<(Accel, Gyro, Option<Mag>), Error<I>> {
        collect_mean_values(self, delay, accel_scale, gravity).await
    }

    /// Perform a full device calibration (note that this function might never terminate).
    ///
    /// A higher level building block for performing calibration,
    /// this function should be called repeatedly, until `is_empty()` returns `true` on the
    /// returned `CalibrationActions`.
    ///
    /// At each iteration the calibration gets "better", and it is considered completed when
    /// there are no more actions to perform.
    ///
    /// This is an example of how a calibration loop can be written:
    ///
    /// ```ignore
    ///     let calibration_parameters =
    ///         CalibrationParameters::new(ACCEL_SCALE, GYRO_SCALE, ReferenceGravity::ZN);
    ///     let mut actions = CalibrationActions::all();
    ///     let mut accel_error: Accel;
    ///     let mut gyro_error: Gyro;
    ///     while !actions.is_empty() {
    ///         (actions, accel_error, gyro_error) = mpu
    ///             .calibration_loop(&mut delay, &calibration_parameters, actions)
    ///             .unwrap();
    ///         uprintln!(
    ///             serial,
    ///             "errors: [a {} {} {}] [g {} {} {}] [done: {}]",
    ///             accel_error.x(),
    ///             accel_error.y(),
    ///             accel_error.z(),
    ///             gyro_error.x(),
    ///             gyro_error.y(),
    ///             gyro_error.z(),
    ///             actions.is_empty(),
    ///         );
    ///     }
    ///     let accel_offset = mpu.get_accel_calibration().unwrap();
    ///     let gyro_offset = mpu.get_gyro_calibration().unwrap();
    ///     uprintln!(
    ///         serial,
    ///         "offsets: [a {} {} {}] [g {} {} {}]",
    ///         accel_offset.x(),
    ///         accel_offset.y(),
    ///         accel_offset.z(),
    ///         gyro_offset.x(),
    ///         gyro_offset.y(),
    ///         gyro_offset.z(),
    ///     );
    /// ```
    ///
    /// When the loop is done the offsets can be collected and hardcoded in the initialization
    /// of the application that will use this device with exactly these settings.
    #[cfg(not(feature = "mpu9265"))]
    pub async fn calibration_loop(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
        actions: CalibrationActions,
    ) -> Result<(CalibrationActions, Accel, Gyro), Error<I>> {
        calibration_loop(self, delay, parameters, actions).await
    }

    #[cfg(feature = "mpu9265")]
    pub async fn calibration_loop(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
        actions: CalibrationActions,
    ) -> Result<(CalibrationActions, Accel, Gyro, Option<Mag>), Error<I>> {
        calibration_loop(self, delay, parameters, actions).await
    }

    pub async fn get_accel_calibration(&mut self) -> Result<Accel, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::AccelOffsetX_H, &mut data)
            .await?;
        Ok(Accel::from_bytes(data))
    }

    pub async fn get_gyro_calibration(&mut self) -> Result<Gyro, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::GyroOffsetX_H, &mut data)
            .await?;
        Ok(Gyro::from_bytes(data))
    }

    pub async fn set_accel_calibration(&mut self, values: &Accel) -> Result<(), Error<I>> {
        let data = values.to_bytes();
        let bytes = [
            Register::AccelOffsetX_H as u8,
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
        ];
        self.write(&bytes).await
    }

    pub async fn set_gyro_calibration(&mut self, values: &Gyro) -> Result<(), Error<I>> {
        let data = values.to_bytes();
        let bytes = [
            Register::GyroOffsetX_H as u8,
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
        ];
        self.write(&bytes).await
    }

    pub async fn set_accel_full_scale(&mut self, scale: AccelFullScale) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::AccelConfig).await?;
        value |= (scale as u8) << 3;
        self.write_register(Register::AccelConfig, value).await
    }

    pub async fn set_gyro_full_scale(&mut self, scale: GyroFullScale) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::GyroConfig).await?;
        value |= (scale as u8) << 3;
        self.write_register(Register::GyroConfig, value).await
    }

    /// Sets the sample rate divider to control sensor output data rate.
    ///
    /// The sensor's output data rate is calculated as:
    /// Sample Rate = Gyroscope Output Rate / (1 + divider)
    ///
    /// Where Gyroscope Output Rate is:
    /// - 8kHz when DLPF is disabled (filter 0 or 7)
    /// - 1kHz when DLPF is enabled (filter 1-6)
    ///
    /// Common divider values:
    /// - 0: Maximum rate (8kHz or 1kHz)
    /// - 4: 200Hz (good for motion tracking)
    /// - 9: 100Hz (good balance of speed/power)
    /// - 99: 10Hz (power saving mode)
    ///
    /// Note: Lower sample rates reduce power consumption but increase latency.
    pub async fn set_sample_rate_divider(&mut self, div: u8) -> Result<(), Error<I>> {
        self.write_register(Register::SmpRtDiv, div).await
    }

    /// Configures the Digital Low Pass Filter (DLPF) to reduce noise in sensor readings.
    ///
    /// The DLPF helps remove:
    /// - High frequency vibrations
    /// - Electrical noise
    /// - Motion artifacts
    ///
    /// Filter options balance between:
    /// - Noise reduction (higher filtering)
    /// - Latency (higher filtering = more delay)
    /// - Bandwidth (lower filtering = more responsive)
    pub async fn set_digital_lowpass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::Config).await?;
        value |= filter as u8;
        self.write_register(Register::Config, value).await
    }

    pub async fn reset_fifo(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 2;
        self.write_register(Register::UserCtrl, value).await
    }

    pub async fn enable_fifo(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 6;
        self.write_register(Register::UserCtrl, value).await
    }

    /// Set the DMP bit.
    /// To perform full DMP initialization, see `initialize_dmp()`
    pub async fn enable_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 7;
        self.write_register(Register::UserCtrl, value).await
    }

    // Unset the DMP bit.
    pub async fn disable_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value &= !(1 << 7);
        self.write_register(Register::UserCtrl, value).await
    }

    /// Reset the DMP processor
    pub async fn reset_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 3;
        self.write_register(Register::UserCtrl, value).await
    }

    /// Read the FIFO
    pub async fn read_fifo<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a [u8], Error<I>> {
        let mut len = self.get_fifo_count().await?;

        if buf.len() < len {
            len = buf.len();
        }

        if len == 0 {
            Ok(&buf[0..0])
        } else {
            self.read_registers(Register::FifoRw, &mut buf[0..len])
                .await
        }
    }

    pub async fn get_fifo_enabled(&mut self) -> Result<Fifo, Error<I>> {
        let value = self.read_register(Register::FifoEn).await?;
        Ok(Fifo::from_byte(value))
    }

    pub async fn set_fifo_enabled(&mut self, fifo: Fifo) -> Result<(), Error<I>> {
        self.write_register(Register::FifoEn, fifo.to_byte()).await
    }

    pub async fn get_fifo_count(&mut self) -> Result<usize, Error<I>> {
        let mut buf = [0; 2];
        let _value = self.read_registers(Register::FifoCount_H, &mut buf).await?;
        Ok(u16::from_be_bytes(buf) as usize)
    }

    pub async fn disable_sleep(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1).await?;
        value &= !(1 << 6);
        self.write_register(Register::PwrMgmt1, value).await
    }

    pub async fn accel(&mut self) -> Result<Accel, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::AccelX_H, &mut data).await?;
        Ok(Accel::from_bytes(data))
    }

    pub async fn gyro(&mut self) -> Result<Gyro, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::GyroX_H, &mut data).await?;
        Ok(Gyro::from_bytes(data))
    }

    /// Gets the 6 degrees of freedom at once - Acceleration and Gyroscope.
    /// Reads both accelerometer and gyroscope data in a single operation.
    ///
    /// This method is more efficient than separate accel() and gyro() calls because:
    /// 1. It requires only one I2C transaction
    /// 2. The readings are synchronized (taken at the same time)
    /// 3. Reduces timing jitter between acceleration and rotation data
    ///
    /// Returns:
    /// - Accelerometer data (x,y,z) in g-forces
    /// - Gyroscope data (x,y,z) in degrees/second
    ///
    /// Note: The values depend on the current full-scale settings
    /// configured via set_accel_full_scale() and set_gyro_full_scale()
    pub async fn motion6(&mut self) -> Result<(Accel, Gyro), Error<I>> {
        let mut data = [0; 14];
        self.read_registers(Register::AccelX_H, &mut data).await?;

        let accel = Accel::from_bytes([data[0], data[1], data[2], data[3], data[4], data[5]]);
        let gyro = Gyro::from_bytes([data[8], data[9], data[10], data[11], data[12], data[13]]);
        Ok((accel, gyro))
    }

    /// Read the current temperature from the sensor's internal temperature sensor.
    pub async fn temperature(&mut self) -> Result<Temperature, Error<I>> {
        let mut data = [0; 2];
        self.read_registers(Register::TempOut_H, &mut data).await?;
        Ok(Temperature::from_bytes(data))
    }

    #[cfg(feature = "mpu9265")]
    /// Initialize the AK8963 magnetometer
    pub async fn init_mag(&mut self) -> Result<(), Error<I>> {
        // Enable I2C master mode
        let mut value = self.read_register(Register::UserCtrl).await?;
        value |= 1 << 5;
        self.write_register(Register::UserCtrl, value).await?;

        // Configure I2C master
        self.write_register(Register::I2CMasterCtrl, 0x0D).await?; // 400kHz I2C clock

        // Read sensitivity adjustment values and store them
        let (asa_x, asa_y, asa_z) = self.read_mag_sensitivity().await?;
        let sx = ((asa_x as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
        let sy = ((asa_y as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;
        let sz = ((asa_z as f32 - 128.0) * 0.5 / 128.0 + 1.0) as f32;

        // Configure magnetometer for continuous measurement mode with 16-bit output
        let config = MagMode::Continuous2 as u8 | (MagBitMode::Bit16 as u8) << 4;
        self.write_mag_register(MagRegister::Control1, config)
            .await?;

        // Initialize calibration with sensitivity adjustments but no offsets
        self.mag_calibration = Some(crate::sensor_mpu9265::MagCalibration {
            offsets: (0, 0, 0),
            sensitivity: (sx, sy, sz),
        });

        Ok(())
    }

    #[cfg(feature = "mpu9265")]
    /// Read magnetometer sensitivity adjustment values from Fuse ROM
    async fn read_mag_sensitivity(&mut self) -> Result<(u8, u8, u8), Error<I>> {
        // Enter Fuse ROM access mode
        self.write_mag_register(MagRegister::Control1, MagMode::FuseROM as u8)
            .await?;

        // Read sensitivity adjustment values
        let asa_x = self.read_mag_register(MagRegister::ASAX).await?;
        let asa_y = self.read_mag_register(MagRegister::ASAY).await?;
        let asa_z = self.read_mag_register(MagRegister::ASAZ).await?;

        Ok((asa_x, asa_y, asa_z))
    }

    #[cfg(feature = "mpu9265")]
    /// Read magnetometer measurements with calibration applied if available
    pub async fn mag(&mut self) -> Result<Mag, Error<I>> {
        let mut data = [0; 6];

        // Wait for data ready
        loop {
            let status = self.read_mag_register(MagRegister::Status1).await?;
            if status & 0x01 != 0 {
                break;
            }
        }

        // Read magnetometer data
        self.read_mag_registers(MagRegister::MeasurementXL, &mut data)
            .await?;

        // Check for overflow
        let status2 = self.read_mag_register(MagRegister::Status2).await?;
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

    #[cfg(feature = "mpu9265")]
    /// Read from magnetometer register
    async fn read_mag_register(&mut self, reg: MagRegister) -> Result<u8, Error<I>> {
        // Set slave 0 to read from magnetometer register
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR | 0x80)
            .await?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)
            .await?;
        self.write_register(Register::I2CSlave0Ctrl, 0x81).await?; // Enable read, 1 byte

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus).await?;
            if status & 0x01 == 0 {
                break;
            }
        }

        // Read the data
        self.read_register(Register::I2CSlave0DI).await
    }

    #[cfg(feature = "mpu9265")]
    /// Write to magnetometer register
    async fn write_mag_register(&mut self, reg: MagRegister, value: u8) -> Result<(), Error<I>> {
        // Set slave 0 to write to magnetometer register
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR)
            .await?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)
            .await?;
        self.write_register(Register::I2CSlave0DO, value).await?;
        self.write_register(Register::I2CSlave0Ctrl, 0x81).await?; // Enable write, 1 byte

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus).await?;
            if status & 0x01 == 0 {
                break;
            }
        }

        Ok(())
    }

    #[cfg(feature = "mpu9265")]
    /// Read multiple magnetometer registers
    async fn read_mag_registers(
        &mut self,
        reg: MagRegister,
        data: &mut [u8],
    ) -> Result<(), Error<I>> {
        // Set slave 0 to read from magnetometer registers
        self.write_register(Register::I2CSlave0Addr, MAG_ADDR | 0x80)
            .await?;
        self.write_register(Register::I2CSlave0Reg, reg as u8)
            .await?;
        self.write_register(Register::I2CSlave0Ctrl, 0x80 | data.len() as u8)
            .await?;

        // Wait for transfer to complete
        loop {
            let status = self.read_register(Register::I2CMasterStatus).await?;
            if status & 0x01 == 0 {
                break;
            }
        }

        // Read the data
        for byte in data.iter_mut() {
            *byte = self.read_register(Register::I2CSlave0DI).await?;
        }

        Ok(())
    }

    /// Configure motion detection parameters.
    ///
    /// The MPU-6050's hardware motion detection compares consecutive accelerometer samples
    /// to detect movement. When the difference between samples exceeds the threshold for
    /// the specified duration, a motion event is triggered.
    ///
    /// Configuration process:
    /// 1. Sets motion threshold (1 LSB = 2mg)
    /// 2. Sets minimum duration of motion
    /// 3. Enables detection on all axes
    /// 4. Configures high-pass filter to remove gravity bias
    ///
    /// For maximum sensitivity:
    /// - Use threshold = 1 (2mg) to detect subtle movements
    /// - Use duration = 1 (1ms at 1kHz) for fast response
    /// - Enable motion detection on all axes
    /// - Enable high-pass filter to isolate motion from gravity
    ///
    /// For more stable detection:
    /// - Increase threshold (4-10) to ignore minor vibrations
    /// - Increase duration (5-20) to require sustained motion
    /// - Adjust DLPF settings to filter noise
    ///
    /// Note: The effectiveness of motion detection depends on:
    /// - Proper sensor calibration
    /// - Sample rate configuration
    /// - Digital low-pass filter (DLPF) settings
    pub async fn configure_motion_detection(
        &mut self,
        config: &MotionConfig,
    ) -> Result<(), Error<I>> {
        // Set motion detection threshold
        self.write_register(Register::MotionThreshold, config.threshold)
            .await?;

        // Set motion detection duration
        self.write_register(Register::MotionDuration, config.duration)
            .await?;

        // Configure motion detection control:
        // Bit 7 - Enable motion detection of all axes
        // Bit 6 - Enable motion detection of all axes
        // Bit 5 - Enable motion detection of all axes
        // Bit 4 - Enable motion detection of all axes
        // Bit 3 - Enable motion detection of all axes
        // Bit 2 - Enable motion detection of all axes
        // Bit 1:0 - Reserved
        // Setting 0xFC: Enable motion detection on all axes
        self.write_register(Register::MotionDetectCtrl, 0xFC)
            .await?;

        // Configure accelerometer HPF for motion detection
        let mut config = self.read_register(Register::AccelConfig).await?;
        config |= 0x07; // Enable HPF, set to maximum bandwidth
        self.write_register(Register::AccelConfig, config).await?;

        Ok(())
    }

    /// Enable motion detection interrupt.
    ///
    /// After enabling, use `wait_for_motion()` to asynchronously wait for motion events.
    pub async fn enable_motion_interrupt(&mut self) -> Result<(), Error<I>> {
        // Enable motion interrupt (bit 6)
        let mut value = self.read_register(Register::IntEnable).await?;
        value |= 1 << 6;
        self.write_register(Register::IntEnable, value).await
    }

    /// Disable motion detection interrupt.
    pub async fn disable_motion_interrupt(&mut self) -> Result<(), Error<I>> {
        // Disable motion interrupt (bit 6)
        let mut value = self.read_register(Register::IntEnable).await?;
        value &= !(1 << 6);
        self.write_register(Register::IntEnable, value).await
    }

    /// Check if motion is currently detected
    pub async fn check_motion(&mut self) -> Result<MotionDetected, Error<I>> {
        let status = self.read_register(Register::MotionDetectStatus).await?;
        Ok(status.into())
    }

    /// Poll for motion detection via I2C.
    ///
    /// This method continuously polls the interrupt status register via I2C until motion is detected.
    /// While functional, this is not the most efficient approach. For better power efficiency,
    /// consider using the MPU6050's hardware interrupt pin instead.
    ///
    /// The hardware interrupt approach:
    /// 1. Connect MPU6050 INT pin to a microcontroller GPIO configured as interrupt input
    /// 2. Configure motion detection and enable the interrupt
    /// 3. Put the microcontroller into sleep mode
    /// 4. Let the INT pin wake the microcontroller when motion occurs
    ///
    /// Make sure to call `configure_motion_detection()` and `enable_motion_interrupt()`
    /// before using either polling or hardware interrupt methods.
    ///
    /// # Example of polling approach (not recommended for power efficiency)
    /// ```ignore
    /// // Configure motion detection
    /// let config = MotionConfig {
    ///     threshold: 20,  // 40mg threshold
    ///     duration: 5,    // ~5ms at 1kHz sample rate
    /// };
    /// mpu.configure_motion_detection(&config).await?;
    /// mpu.enable_motion_interrupt().await?;
    ///
    /// // Poll for motion (consider using hardware interrupt instead)
    /// loop {
    ///     if mpu.poll_for_motion(&mut delay).await?.0 {
    ///         println!("Motion detected!");
    ///     }
    /// }
    /// ```
    pub async fn poll_for_motion(
        &mut self,
        delay: &mut impl delay::DelayNs,
    ) -> Result<MotionDetected, Error<I>> {
        // Wait for motion interrupt
        while (self.read_register(Register::IntStatus).await? & (1 << 6)) == 0 {
            // Yield to other tasks while waiting
            delay.delay_us(1).await;
        }

        // Clear interrupt and return motion detected
        Ok(MotionDetected(true))
    }
}
