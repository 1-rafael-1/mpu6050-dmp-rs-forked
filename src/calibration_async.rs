//! Asynchronous Calibration Implementation
//!
//! This module provides async versions of the calibration routines:
//! - Collecting sensor readings asynchronously
//! - Computing calibration offsets without blocking
//! - Adjusting sensor calibration with non-blocking I2C operations
//!
//! The calibration process runs in a loop until all axes are within
//! their specified thresholds, using async/await for efficient I/O.

use crate::{
    accel::{Accel, AccelFullScale},
    calibration::{
        CalibrationActions, CalibrationParameters, MeanAccumulator, ReferenceGravity, DELAY_MS,
        ITERATIONS, WARMUP_ITERATIONS,
    },
    error_async::Error,
    gyro::Gyro,
    sensor_async::Mpu6050,
};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

/// Perform a single loop computing the means of the readings
pub async fn collect_mean_values<I>(
    mpu: &mut Mpu6050<I>,
    delay: &mut impl DelayNs,
    accel_scale: AccelFullScale,
    gravity: ReferenceGravity,
) -> Result<(Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut accumulator = MeanAccumulator::new(accel_scale, gravity);

    #[cfg(feature = "defmt-03")]
    defmt::info!("Starting {} warmup iterations", WARMUP_ITERATIONS);

    for i in 0..WARMUP_ITERATIONS {
        _ = mpu.accel().await?;
        _ = mpu.gyro().await?;
        delay.delay_ms(DELAY_MS).await;

        #[cfg(feature = "defmt-03")]
        if i % 10 == 0 {
            defmt::info!("Warmup progress: {}/{}", i, WARMUP_ITERATIONS);
        }
    }

    #[cfg(feature = "defmt-03")]
    defmt::info!("Starting {} measurement iterations", ITERATIONS);

    for i in 0..ITERATIONS {
        let accel = mpu.accel().await?;
        let gyro = mpu.gyro().await?;
        accumulator.add(&accel, &gyro);
        delay.delay_ms(DELAY_MS).await;

        #[cfg(feature = "defmt-03")]
        if i % 50 == 0 {
            defmt::info!("Measurement progress: {}/{}", i, ITERATIONS);
        }
    }

    #[cfg(feature = "defmt-03")]
    defmt::info!("Computing means from collected samples");

    Ok(accumulator.means())
}

/// A single, full-fledged calibration loop (it also alters the device offset)
pub async fn calibration_loop<I>(
    mpu: &mut Mpu6050<I>,
    delay: &mut impl DelayNs,
    parameters: &CalibrationParameters,
    actions: CalibrationActions,
) -> Result<(CalibrationActions, Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut actions = actions;

    mpu.set_accel_full_scale(parameters.accel_scale).await?;
    mpu.set_gyro_full_scale(parameters.gyro_scale).await?;
    let mut accel_offset = mpu.get_accel_calibration().await?;
    let mut gyro_offset = mpu.get_gyro_calibration().await?;

    #[cfg(feature = "defmt-03")]
    defmt::info!(
        "Starting calibration loop with offsets - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
        accel_offset.x(),
        accel_offset.y(),
        accel_offset.z(),
        gyro_offset.x(),
        gyro_offset.y(),
        gyro_offset.z()
    );

    let (accel_mean, gyro_mean) =
        collect_mean_values(mpu, delay, parameters.accel_scale, parameters.gravity).await?;

    #[cfg(feature = "defmt-03")]
    defmt::info!(
        "Mean values - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
        accel_mean.x(),
        accel_mean.y(),
        accel_mean.z(),
        gyro_mean.x(),
        gyro_mean.y(),
        gyro_mean.z()
    );

    if parameters.accel_threshold.is_value_within(accel_mean.x()) {
        actions = actions.with_accel_x(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Accel X within threshold");
    }
    if parameters.accel_threshold.is_value_within(accel_mean.y()) {
        actions = actions.with_accel_y(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Accel Y within threshold");
    }
    if parameters.accel_threshold.is_value_within(accel_mean.z()) {
        actions = actions.with_accel_z(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Accel Z within threshold");
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.x()) {
        actions = actions.with_gyro_x(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Gyro X within threshold");
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.y()) {
        actions = actions.with_gyro_y(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Gyro Y within threshold");
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.z()) {
        actions = actions.with_gyro_z(false);
        #[cfg(feature = "defmt-03")]
        defmt::info!("Gyro Z within threshold");
    }

    if actions.accel_x() {
        accel_offset.x = parameters
            .accel_threshold
            .next_offset(accel_mean.x(), accel_offset.x());
    }
    if actions.accel_y() {
        accel_offset.y = parameters
            .accel_threshold
            .next_offset(accel_mean.y(), accel_offset.y());
    }
    if actions.accel_z() {
        accel_offset.z = parameters
            .accel_threshold
            .next_offset(accel_mean.z(), accel_offset.z());
    }
    if actions.gyro_x() {
        gyro_offset.x = parameters
            .gyro_threshold
            .next_offset(gyro_mean.x(), gyro_offset.x());
    }
    if actions.gyro_y() {
        gyro_offset.y = parameters
            .gyro_threshold
            .next_offset(gyro_mean.y(), gyro_offset.y());
    }
    if actions.gyro_z() {
        gyro_offset.z = parameters
            .gyro_threshold
            .next_offset(gyro_mean.z(), gyro_offset.z());
    }

    if !actions.is_empty() {
        #[cfg(feature = "defmt-03")]
        defmt::info!(
            "Setting new offsets - Accel: ({}, {}, {}), Gyro: ({}, {}, {})",
            accel_offset.x(),
            accel_offset.y(),
            accel_offset.z(),
            gyro_offset.x(),
            gyro_offset.y(),
            gyro_offset.z()
        );
        mpu.set_accel_calibration(&accel_offset).await?;
        mpu.set_gyro_calibration(&gyro_offset).await?;
    }

    Ok((actions, accel_mean, gyro_mean))
}

/// Repeatedly perform calibration loops until the errors are within the given thresholds
pub async fn calibrate<I>(
    mpu: &mut Mpu6050<I>,
    delay: &mut impl DelayNs,
    parameters: &CalibrationParameters,
) -> Result<(Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut actions = CalibrationActions::all();
    while !actions.is_empty() {
        (actions, _, _) = calibration_loop(mpu, delay, parameters, actions).await?;
    }
    Ok((
        mpu.get_accel_calibration().await?,
        mpu.get_gyro_calibration().await?,
    ))
}
