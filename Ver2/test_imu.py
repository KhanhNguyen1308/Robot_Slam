#!/usr/bin/env python3
"""
Test script for MPU6050 IMU
Verifies I2C connection, calibration, and data reading
"""
import sys
import time
import numpy as np
import logging

# Import MPU6050 driver
from mpu6050_imu import MPU6050, ComplementaryFilter

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)


def test_basic_connection():
    """Test basic I2C connection to MPU6050"""
    logger.info("=" * 60)
    logger.info("Test 1: Basic Connection")
    logger.info("=" * 60)
    
    imu = MPU6050(bus_number=1, address=0x68)
    
    if not imu.connect():
        logger.error("Failed to connect to MPU6050")
        logger.error("Check:")
        logger.error("  1. MPU6050 is connected to I2C bus 1")
        logger.error("  2. Wiring: SDA to pin 3, SCL to pin 5, VCC to 3.3V, GND to GND")
        logger.error("  3. I2C is enabled: sudo i2cdetect -y -r 1")
        return False
    
    logger.info("✓ MPU6050 connected successfully!")
    imu.disconnect()
    return True


def test_raw_readings():
    """Test raw sensor readings"""
    logger.info("\n" + "=" * 60)
    logger.info("Test 2: Raw Sensor Readings")
    logger.info("=" * 60)
    
    imu = MPU6050(bus_number=1, address=0x68, gyro_range=250, accel_range=2)
    
    if not imu.connect():
        logger.error("Failed to connect")
        return False
    
    logger.info("Reading 10 samples (uncalibrated)...")
    
    for i in range(10):
        accel = imu.read_accel()
        gyro = imu.read_gyro()
        temp = imu.read_temperature()
        
        logger.info(f"Sample {i+1}:")
        logger.info(f"  Accel: [{accel[0]:6.3f}, {accel[1]:6.3f}, {accel[2]:6.3f}] g")
        logger.info(f"  Gyro:  [{gyro[0]:7.2f}, {gyro[1]:7.2f}, {gyro[2]:7.2f}] °/s")
        logger.info(f"  Temp:  {temp:.1f} °C")
        
        time.sleep(0.5)
    
    imu.disconnect()
    logger.info("✓ Raw readings successful!")
    return True


def test_calibration():
    """Test IMU calibration"""
    logger.info("\n" + "=" * 60)
    logger.info("Test 3: Calibration")
    logger.info("=" * 60)
    logger.info("IMPORTANT: Keep the IMU stationary during calibration!")
    logger.info("Press Enter to start calibration...")
    input()
    
    imu = MPU6050(bus_number=1, address=0x68, gyro_range=250, accel_range=2)
    
    if not imu.connect():
        logger.error("Failed to connect")
        return False
    
    # Calibrate
    imu.calibrate(samples=500)
    
    # Test calibrated readings
    logger.info("\nReading 10 calibrated samples...")
    
    for i in range(10):
        accel = imu.read_accel()
        gyro = imu.read_gyro()
        
        logger.info(f"Sample {i+1}:")
        logger.info(f"  Accel: [{accel[0]:6.3f}, {accel[1]:6.3f}, {accel[2]:6.3f}] g")
        logger.info(f"  Gyro:  [{gyro[0]:7.2f}, {gyro[1]:7.2f}, {gyro[2]:7.2f}] °/s")
        
        time.sleep(0.5)
    
    logger.info("\nExpected when stationary:")
    logger.info("  - Accel Z should be close to 1.0 g (gravity)")
    logger.info("  - Accel X, Y should be close to 0.0 g")
    logger.info("  - All gyro values should be close to 0.0 °/s")
    
    imu.disconnect()
    logger.info("✓ Calibration test complete!")
    return True


def test_orientation_integration():
    """Test orientation integration from gyroscope"""
    logger.info("\n" + "=" * 60)
    logger.info("Test 4: Orientation Integration")
    logger.info("=" * 60)
    logger.info("Rotate the IMU and watch the orientation update")
    logger.info("Press Enter to start (Ctrl+C to stop)...")
    input()
    
    imu = MPU6050(bus_number=1, address=0x68, gyro_range=250, accel_range=2)
    
    if not imu.connect():
        logger.error("Failed to connect")
        return False
    
    # Calibrate
    logger.info("Calibrating (keep stationary)...")
    imu.calibrate(samples=300)
    
    # Start continuous reading
    imu.start_reading()
    
    logger.info("\nOrientation tracking started (press Ctrl+C to stop):")
    logger.info("Rotate the sensor and observe yaw/pitch/roll changes")
    
    try:
        while True:
            yaw, pitch, roll = imu.get_orientation()
            accel = imu.get_accel()
            gyro = imu.get_gyro()
            
            logger.info(f"Orientation: Yaw={np.degrees(yaw):6.1f}° Pitch={np.degrees(pitch):6.1f}° Roll={np.degrees(roll):6.1f}° | "
                       f"Gyro Z={gyro[2]:6.1f}°/s")
            
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        logger.info("\nStopping...")
    
    imu.disconnect()
    logger.info("✓ Orientation integration test complete!")
    return True


def test_complementary_filter():
    """Test complementary filter for orientation"""
    logger.info("\n" + "=" * 60)
    logger.info("Test 5: Complementary Filter")
    logger.info("=" * 60)
    logger.info("Tests fusion of accelerometer and gyroscope")
    logger.info("Press Enter to start (Ctrl+C to stop)...")
    input()
    
    imu = MPU6050(bus_number=1, address=0x68, gyro_range=250, accel_range=2)
    
    if not imu.connect():
        logger.error("Failed to connect")
        return False
    
    # Calibrate
    logger.info("Calibrating (keep stationary)...")
    imu.calibrate(samples=300)
    
    # Create complementary filter
    comp_filter = ComplementaryFilter(alpha=0.98)
    
    logger.info("\nComplementary filter started (press Ctrl+C to stop):")
    
    last_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            accel = imu.read_accel()
            gyro = imu.read_gyro()
            gyro_rad = np.radians(gyro)
            
            # Update filter
            comp_filter.update(accel, gyro_rad, dt)
            
            yaw, pitch, roll = comp_filter.get_orientation()
            
            logger.info(f"Fused Orientation: Yaw={np.degrees(yaw):6.1f}° "
                       f"Pitch={np.degrees(pitch):6.1f}° Roll={np.degrees(roll):6.1f}°")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("\nStopping...")
    
    imu.disconnect()
    logger.info("✓ Complementary filter test complete!")
    return True


def main():
    """Run all IMU tests"""
    logger.info("MPU6050 IMU Test Suite")
    logger.info("Testing MPU6050 on I2C bus 1 (Jetson Nano)")
    logger.info("")
    
    tests = [
        ("Basic Connection", test_basic_connection),
        ("Raw Readings", test_raw_readings),
        ("Calibration", test_calibration),
        ("Orientation Integration", test_orientation_integration),
        ("Complementary Filter", test_complementary_filter),
    ]
    
    # Ask which test to run
    logger.info("Available tests:")
    for i, (name, _) in enumerate(tests):
        logger.info(f"  {i+1}. {name}")
    logger.info(f"  {len(tests)+1}. Run all tests")
    
    try:
        choice = input("\nSelect test (1-{}): ".format(len(tests)+1))
        choice = int(choice)
        
        if choice == len(tests) + 1:
            # Run all tests
            for name, test_func in tests:
                logger.info(f"\n{'='*60}")
                logger.info(f"Running: {name}")
                logger.info(f"{'='*60}")
                if not test_func():
                    logger.error(f"Test '{name}' failed!")
                    return 1
        elif 1 <= choice <= len(tests):
            # Run selected test
            name, test_func = tests[choice - 1]
            if not test_func():
                logger.error(f"Test '{name}' failed!")
                return 1
        else:
            logger.error("Invalid choice")
            return 1
            
    except ValueError:
        logger.error("Invalid input")
        return 1
    except KeyboardInterrupt:
        logger.info("\nTest interrupted by user")
        return 1
    
    logger.info("\n" + "=" * 60)
    logger.info("All tests completed successfully! ✓")
    logger.info("=" * 60)
    logger.info("\nYour MPU6050 is ready to use with the robot system!")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
