//! MPU6050-DMP Quaternion Example
//!
//! Understanding Quaternions:
//! Quaternions are a mathematical way to represent 3D rotations that avoid the problems
//! of gimbal lock that can occur with Euler angles (Yaw, Pitch, Roll). A quaternion
//! consists of four components (w, x, y, z):
//!
//! - w: The scalar component, represents the amount of rotation
//! - x, y, z: The vector components, represent the axis of rotation
//!
//! For a quaternion representing orientation:
//! - When the sensor is level, w will be close to 1, and x,y,z close to 0
//! - As the sensor rotates:
//!   * x component increases/decreases with roll (side-to-side tilt)
//!   * y component increases/decreases with pitch (forward-backward tilt)
//!   * z component increases/decreases with yaw (rotation around vertical axis)
//!
//! The DMP (Digital Motion Processor) on the MPU6050:
//! - Handles complex sensor fusion calculations internally
//! - Combines accelerometer and gyroscope data
//! - Outputs quaternions representing absolute orientation
//! - Automatically converts to Yaw, Pitch, Roll angles
//!
//! Yaw, Pitch, Roll Explanation:
//! - Yaw: Rotation around vertical axis (like shaking head "no")
//! - Pitch: Forward/backward tilt (like nodding "yes")
//! - Roll: Side-to-side tilt (like tilting head toward shoulder)
//!
//! Hardware Setup:
//! - Connect MPU6050 to Raspberry Pi Pico:
//!   - SDA -> GP14
//!   - SCL -> GP15
//!   - VCC -> 3.3V
//!   - GND -> GND

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config, i2c::InterruptHandler};
use embassy_time::{Delay, Timer};
use {defmt_rtt as _, panic_probe as _};

// mpu6050-dmp
use mpu6050_dmp::{
    address::Address, calibration::CalibrationParameters, quaternion::Quaternion,
    sensor_async::Mpu6050, yaw_pitch_roll::YawPitchRoll,
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
    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);
    let mut sensor = Mpu6050::new(bus, Address::default()).await.unwrap();
    let mut delay = Delay;

    info!("MPU6050-DMP Sensor Initialized");

    // Initialize DMP
    info!("Initializing DMP");
    sensor.initialize_dmp(&mut delay).await.unwrap();

    // Configure calibration parameters
    let calibration_params = CalibrationParameters::new(
        mpu6050_dmp::accel::AccelFullScale::G2,
        mpu6050_dmp::gyro::GyroFullScale::Deg2000,
        mpu6050_dmp::calibration::ReferenceGravity::ZN,
    );

    // info!("Calibrating Sensor");
    // sensor
    //     .calibrate(&mut delay, &calibration_params)
    //     .await
    //     .unwrap();
    // info!("Sensor Calibrated");

    // Configure DMP update rate
    sensor.set_sample_rate_divider(9).await.unwrap(); // 100Hz
    info!("Sample rate configured");

    // Enable FIFO for quaternion data
    sensor.enable_fifo().await.unwrap();
    info!("FIFO enabled for quaternion data");

    // Buffer for FIFO data (DMP packets are 28 bytes)
    let mut buffer = [0u8; 28];

    // Main loop reading quaternion data
    loop {
        let fifo_count = sensor.get_fifo_count().await.unwrap();

        if fifo_count >= 28 {
            // Read a complete DMP packet
            let data = sensor.read_fifo(&mut buffer).await.unwrap();

            // First 16 bytes contain quaternion data
            // The quaternion represents the sensor's orientation in 3D space:
            // - w: cos(angle/2) - indicates amount of rotation
            // - x,y,z: axis * sin(angle/2) - indicates rotation axis
            let quat = Quaternion::from_bytes(&data[..16]).unwrap().normalize();

            // Convert quaternion to more intuitive Yaw, Pitch, Roll angles
            // Note: angles are in radians (-π to π)
            let ypr = YawPitchRoll::from(quat);

            // Display quaternion components
            // Values are normalized (sum of squares = 1)
            info!("Orientation:");
            info!(
                "  Quaternion: w={}, x={}, y={}, z={}",
                quat.w, quat.x, quat.y, quat.z
            );

            // Convert radians to degrees for more intuitive reading
            let yaw_deg = ypr.yaw * 180.0 / core::f32::consts::PI;
            let pitch_deg = ypr.pitch * 180.0 / core::f32::consts::PI;
            let roll_deg = ypr.roll * 180.0 / core::f32::consts::PI;

            info!(
                "  Angles [deg]: yaw={}, pitch={}, roll={}",
                yaw_deg, pitch_deg, roll_deg
            );
        }

        Timer::after_millis(10).await;
    }
}
