//! MPU6050 Motion Detection Example using Hardware Interrupt
//!
//! This example demonstrates how to use the MPU6050's hardware motion detection feature
//! with its INT pin for efficient, low-power movement detection. The implementation:
//!
//! 1. Configures the sensor for motion detection:
//!    - Uses 2mg threshold for motion detection sensitivity
//!    - Sets 1ms duration for motion event qualification
//!    - Monitors acceleration on all axes
//!    - Uses digital low-pass filter (DLPF) to reduce noise
//!
//! 2. Configures sensor settings:
//!    - Sets accelerometer range to ±2g for maximum sensitivity
//!    - Uses X-axis gyroscope as clock source for stability
//!    - Configures 1kHz sampling rate
//!    - Applies calibration for accurate readings
//!
//! 3. Implements interrupt-based detection:
//!    - Uses MPU6050's INT pin to signal motion events
//!    - Monitors GPIO input for interrupt signal
//!    - Samples motion data at 50ms intervals during activity
//!    - Detects both motion start and stop conditions
//!
//! This configuration balances sensitivity and reliability.
//! Adjustable parameters include:
//! - Motion threshold (currently 2mg)
//! - Detection duration (currently 1ms)
//! - Sample rate divider (currently set for 1kHz)
//! - Digital low-pass filter settings
//!
//! Hardware Setup:
//! - Connect MPU6050 to Raspberry Pi Pico:
//!   - SDA -> GP14 (I2C1 data line)
//!   - SCL -> GP15 (I2C1 clock line)
//!   - INT -> GP16 (Motion interrupt input)
//!   - VCC -> 3.3V
//!   - GND -> GND

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    block::ImageDef,
    config::Config,
    gpio::{Input, Pull},
    i2c::InterruptHandler,
};
use embassy_time::{Delay, Timer};
use {defmt_rtt as _, panic_probe as _};

// mpu6050-dmp
use mpu6050_dmp::{
    address::Address, calibration::CalibrationParameters, motion::MotionConfig,
    sensor_async::Mpu6050,
};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize I2C and sensor
    let sda = p.PIN_14;
    let scl = p.PIN_15;

    // Configure GPIO16 as interrupt input with pull-up
    let mut motion_int = Input::new(p.PIN_16, Pull::Up);

    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);
    let mut sensor = Mpu6050::new(bus, Address::default()).await.unwrap();
    let mut delay = Delay;

    info!("MPU6050-DMP Sensor Initialized");

    // Configure sensor settings
    sensor
        .set_clock_source(mpu6050_dmp::clock_source::ClockSource::Xgyro)
        .await
        .unwrap();

    // Set accelerometer full scale to most sensitive range
    sensor
        .set_accel_full_scale(mpu6050_dmp::accel::AccelFullScale::G2)
        .await
        .unwrap();

    // Configure DLPF for maximum sensitivity
    sensor
        .set_digital_lowpass_filter(mpu6050_dmp::config::DigitalLowPassFilter::Filter6)
        .await
        .unwrap();

    // Set sample rate to 1kHz (1ms period)
    sensor.set_sample_rate_divider(0).await.unwrap();

    // Configure calibration parameters with more forgiving thresholds
    // Since we're using G2 scale, gravity is ~16384, so threshold of 128 is <1% error
    let calibration_params = CalibrationParameters::new(
        mpu6050_dmp::accel::AccelFullScale::G8,
        mpu6050_dmp::gyro::GyroFullScale::Deg1000,
        mpu6050_dmp::calibration::ReferenceGravity::ZP, // Z-axis pointing up
    )
    .with_accel_threshold(128) // About 0.8% of full scale for accelerometer
    .with_gyro_threshold(64) // More forgiving threshold for gyro
    .with_iterations(100) // Reduce number of samples to speed up calibration
    .with_warmup_iterations(10);

    info!("Calibrating Sensor");
    sensor
        .calibrate(&mut delay, &calibration_params)
        .await
        .unwrap();
    info!("Sensor Calibrated");

    // Configure motion detection with maximum sensitivity
    let motion_config = MotionConfig {
        threshold: 1, // 2mg threshold (minimum possible)
        duration: 1,  // 1ms at 1kHz sample rate (fastest response)
    };
    sensor
        .configure_motion_detection(&motion_config)
        .await
        .unwrap();
    sensor.enable_motion_interrupt().await.unwrap();

    // Before entering cyclic measurement, make sure the Interrupt Pin is high
    info!("Starting motion detection");
    motion_int.wait_for_high().await;

    // Main loop monitoring motion detection events
    loop {
        // Wait for hardware interrupt (INT pin going low)
        motion_int.wait_for_low().await;

        let (accel, gyro) = sensor.motion6().await.unwrap();
        report_motion(
            accel.x() as i32,
            accel.y() as i32,
            accel.z() as i32,
            gyro.x() as i32,
            gyro.y() as i32,
            gyro.z() as i32,
        );

        // Monitor motion while it continues
        while sensor.check_motion().await.unwrap().0 {
            // Read current sensor data
            let (accel, gyro) = sensor.motion6().await.unwrap();
            report_motion(
                accel.x() as i32,
                accel.y() as i32,
                accel.z() as i32,
                gyro.x() as i32,
                gyro.y() as i32,
                gyro.z() as i32,
            );

            // Wait 50ms before next check
            Timer::after_millis(50).await;
        }

        // Wait for INT to go high (motion completely stopped)
        // before looking for new motion
        info!("No more motion detected");
        motion_int.wait_for_high().await;
    }
}

fn report_motion(accel_x: i32, accel_y: i32, accel_z: i32, gyro_x: i32, gyro_y: i32, gyro_z: i32) {
    info!(
        "Acceleration [mg]: x={}, y={}, z={}. Gyroscope [deg/s]: x={}, y={}, z={}",
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    );
}
