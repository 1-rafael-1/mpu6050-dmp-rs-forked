//! MPU6050-DMP FIFO Buffer Example
//!
//! This example demonstrates basic FIFO buffer operations with the MPU6050:
//! - Initializing the Digital Motion Processor (DMP)
//! - Enabling the FIFO buffer
//! - Continuously monitoring FIFO count
//! - Reading raw sensor data from FIFO
//! - Handling FIFO overflow conditions
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
use mpu6050_dmp::{address::Address, calibration::CalibrationParameters, sensor_async::Mpu6050};

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

    // Configure FIFO
    sensor.enable_fifo().await.unwrap();
    info!("FIFO enabled");

    // Main loop demonstrating FIFO usage
    let mut buffer = [0u8; 1024]; // Buffer for FIFO data
    loop {
        // Get FIFO count
        let fifo_count = sensor.get_fifo_count().await.unwrap();
        info!("FIFO Count: {} bytes", fifo_count);

        if fifo_count >= 1024 {
            // FIFO is full - reset to prevent overflow
            info!("FIFO full - resetting");
            sensor.reset_fifo().await.unwrap();
            continue;
        }

        if fifo_count > 0 {
            // Read available data from FIFO
            let data = sensor.read_fifo(&mut buffer[..fifo_count]).await.unwrap();
            info!("Read {} bytes from FIFO", data.len());

            // Process the raw FIFO data
            // Note: In a real application, you would parse this data according to
            // the sensor's data format specification
            if data.len() >= 6 {
                // Display first 6 bytes in both decimal and hex format
                info!("First 6 bytes:");
                info!(
                    "  Dec: {}, {}, {}, {}, {}, {}",
                    data[0], data[1], data[2], data[3], data[4], data[5]
                );
                // Note: defmt doesn't support hex formatting, so we'll just show decimal
            }
        }

        // Small delay before next FIFO check
        Timer::after_millis(50).await;
    }
}
