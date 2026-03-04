#!/usr/bin/env python3
"""
Advanced Motor Calibration Using MPU6050 IMU
Accurately measures actual rotation angle to calibrate wheel_base
"""
import sys
import time
import logging
import math
import numpy as np

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# Import modules
try:
    from jetson_motor_controller import JetsonMotorController
    from mpu6050_imu import MPU6050
except ImportError as e:
    logger.error(f"Cannot import required modules: {e}")
    sys.exit(1)


class IMUCalibration:
    """IMU-based motor calibration"""
    
    def __init__(self, imu: MPU6050, motor: JetsonMotorController):
        self.imu = imu
        self.motor = motor
        self.calibration_data = []
    
    def measure_rotation(self, angular_velocity: float, duration: float) -> dict:
        """
        Measure actual rotation using IMU
        
        Args:
            angular_velocity: Command angular velocity (rad/s)
            duration: Duration to rotate (seconds)
        
        Returns:
            dict with commanded_angle, measured_angle, error
        """
        logger.info("Preparing to measure rotation...")
        
        # Reset IMU orientation
        self.imu.reset_orientation()
        time.sleep(0.5)
        
        # Get initial orientation
        initial_data = self.imu.get_orientation()
        if initial_data is None:
            logger.error("Failed to read IMU!")
            return None
        
        initial_yaw = initial_data['yaw']
        logger.info(f"Initial yaw: {math.degrees(initial_yaw):.1f}°")
        
        # Start rotation
        logger.info(f"Rotating at {angular_velocity:.2f} rad/s for {duration:.1f}s...")
        self.motor.set_velocity(0, angular_velocity)
        
        # Sample IMU during rotation
        samples = []
        sample_interval = 0.05  # 20 Hz
        start_time = time.time()
        
        while time.time() - start_time < duration:
            data = self.imu.get_orientation()
            if data:
                elapsed = time.time() - start_time
                samples.append({
                    'time': elapsed,
                    'yaw': data['yaw']
                })
            time.sleep(sample_interval)
        
        # Stop motor
        self.motor.stop()
        time.sleep(0.5)
        
        # Get final orientation
        final_data = self.imu.get_orientation()
        if final_data is None:
            logger.error("Failed to read final IMU!")
            return None
        
        final_yaw = final_data['yaw']
        logger.info(f"Final yaw: {math.degrees(final_yaw):.1f}°")
        
        # Calculate angle change (handle wraparound)
        angle_change = final_yaw - initial_yaw
        
        # Normalize to [-π, π]
        while angle_change > math.pi:
            angle_change -= 2 * math.pi
        while angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        # Expected angle
        expected_angle = angular_velocity * duration
        
        # Calculate error
        error_rad = angle_change - expected_angle
        error_percent = (error_rad / expected_angle * 100) if expected_angle != 0 else 0
        
        result = {
            'commanded_angle_rad': expected_angle,
            'commanded_angle_deg': math.degrees(expected_angle),
            'measured_angle_rad': angle_change,
            'measured_angle_deg': math.degrees(angle_change),
            'error_rad': error_rad,
            'error_deg': math.degrees(error_rad),
            'error_percent': error_percent,
            'samples': samples,
            'angular_velocity': angular_velocity,
            'duration': duration
        }
        
        return result
    
    def calibrate_wheel_base(self, current_wheel_base: float) -> float:
        """
        Calculate corrected wheel_base based on measured rotation
        
        Args:
            current_wheel_base: Current wheel_base value in meters
        
        Returns:
            Corrected wheel_base value
        """
        logger.info("=" * 70)
        logger.info("IMU-BASED WHEEL_BASE CALIBRATION")
        logger.info("=" * 70)
        logger.info("")
        logger.info(f"Current wheel_base: {current_wheel_base*1000:.1f}mm")
        logger.info("")
        
        # Test parameters
        test_angles = [
            (0.5, math.pi),      # 180° at 0.5 rad/s
            (0.3, math.pi/2),    # 90° at 0.3 rad/s
            (-0.3, -math.pi/2)   # -90° at 0.3 rad/s (opposite direction)
        ]
        
        results = []
        
        for i, (angular_vel, target_angle) in enumerate(test_angles, 1):
            logger.info(f"\n--- Test {i}/3: Target {math.degrees(target_angle):.0f}° ---")
            
            duration = abs(target_angle / angular_vel)
            
            input("Press ENTER to start rotation test...")
            
            result = self.measure_rotation(angular_vel, duration)
            
            if result:
                results.append(result)
                
                logger.info("")
                logger.info(f"Commanded: {result['commanded_angle_deg']:.1f}°")
                logger.info(f"Measured:  {result['measured_angle_deg']:.1f}°")
                logger.info(f"Error:     {result['error_deg']:.1f}° ({result['error_percent']:.1f}%)")
            else:
                logger.warning("Test failed, skipping...")
            
            time.sleep(1)
        
        # Calculate correction factor
        if not results:
            logger.error("No valid results!")
            return current_wheel_base
        
        logger.info("")
        logger.info("=" * 70)
        logger.info("CALIBRATION SUMMARY")
        logger.info("=" * 70)
        
        # Calculate average correction factor
        corrections = []
        for result in results:
            # If commanded 180° but only rotated 60°, ratio = 60/180 = 0.333
            # wheel_base needs to be 0.333x smaller
            ratio = result['measured_angle_rad'] / result['commanded_angle_rad']
            corrections.append(ratio)
            logger.info(f"Test: Cmd={result['commanded_angle_deg']:.0f}°, "
                       f"Measured={result['measured_angle_deg']:.0f}°, "
                       f"Ratio={ratio:.3f}")
        
        avg_ratio = np.mean(corrections)
        std_ratio = np.std(corrections)
        
        logger.info("")
        logger.info(f"Average ratio: {avg_ratio:.3f} ± {std_ratio:.3f}")
        logger.info("")
        
        # Calculate new wheel_base
        # If robot rotates less than commanded, wheel_base is too large
        new_wheel_base = current_wheel_base * avg_ratio
        
        logger.info(f"Current wheel_base: {current_wheel_base*1000:.1f}mm")
        logger.info(f"New wheel_base:     {new_wheel_base*1000:.1f}mm")
        logger.info(f"Change:             {(new_wheel_base - current_wheel_base)*1000:.1f}mm")
        logger.info("")
        logger.info("Update main.py config with:")
        logger.info(f"  'wheel_base': {new_wheel_base:.4f},  # {new_wheel_base*1000:.1f}mm")
        logger.info("=" * 70)
        
        return new_wheel_base


def test_imu_rotation():
    """Quick test to verify IMU measures rotation correctly"""
    logger.info("=" * 70)
    logger.info("IMU ROTATION TEST")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This test verifies that IMU can measure rotation.")
    logger.info("You will manually rotate the robot while IMU records the angle.")
    logger.info("")
    
    # Initialize IMU
    imu = MPU6050(bus_number=1, address=0x68)
    if not imu.connect():
        logger.error("Failed to connect to MPU6050!")
        return 1
    
    logger.info("Calibrating IMU - keep robot stationary...")
    imu.calibrate(samples=500)
    imu.start_reading()
    time.sleep(1)
    
    # Reset orientation
    imu.reset_orientation()
    
    input("IMU ready. Manually rotate robot slowly, then press ENTER when done...")
    
    # Read orientation
    data = imu.get_orientation()
    if data:
        logger.info(f"Measured rotation: {math.degrees(data['yaw']):.1f}°")
        logger.info(f"Roll: {math.degrees(data['roll']):.1f}°, Pitch: {math.degrees(data['pitch']):.1f}°")
    else:
        logger.error("Failed to read IMU!")
    
    imu.disconnect()
    return 0


def main():
    """Main calibration workflow"""
    logger.info("=" * 70)
    logger.info("IMU-BASED MOTOR CALIBRATION")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This tool uses MPU6050 to accurately measure rotation angles")
    logger.info("and calibrate the wheel_base parameter.")
    logger.info("")
    
    print("Select mode:")
    print("  1. Test IMU rotation measurement")
    print("  2. Calibrate wheel_base (recommended)")
    
    choice = input("\nChoice (1/2): ")
    
    if choice == '1':
        return test_imu_rotation()
    elif choice == '2':
        pass  # Continue to calibration
    else:
        logger.error("Invalid choice!")
        return 1
    
    # Configuration
    config = {
        'left_step_pin': 33,
        'left_dir_pin': 35,
        'left_enable_pin': 37,
        'right_step_pin': 32,
        'right_dir_pin': 36,
        'right_enable_pin': 38,
        'wheel_diameter': 0.06984,  # 69.84mm (CAD outer diameter, confirmed)
        'wheel_base': 0.2845,       # 284.5mm - will be calibrated
        'steps_per_rev': 200,
        'microsteps': 16,
        'gear_ratio': 38/18        # Motor rotations per wheel rotation (2.111)
    }
    
    # Get current wheel_base
    wheel_base_str = input(f"Enter current wheel_base in mm (or press ENTER for {config['wheel_base']*1000:.1f}mm): ").strip()
    if wheel_base_str:
        config['wheel_base'] = float(wheel_base_str) / 1000.0
    
    gear_ratio_str = input("Enter gear_ratio (or press ENTER for 38/18=2.111): ").strip()
    if gear_ratio_str:
        if '/' in gear_ratio_str:
            parts = gear_ratio_str.split('/')
            config['gear_ratio'] = float(parts[0]) / float(parts[1])
        else:
            config['gear_ratio'] = float(gear_ratio_str)
    
    # Initialize IMU
    logger.info("\nInitializing MPU6050...")
    imu = MPU6050(bus_number=1, address=0x68, gyro_range=250, accel_range=2)
    if not imu.connect():
        logger.error("Failed to connect to MPU6050!")
        return 1
    
    logger.info("Calibrating IMU - keep robot stationary...")
    imu.calibrate(samples=500)
    imu.start_reading()
    time.sleep(1)
    
    # Initialize motor controller
    logger.info("Initializing motor controller...")
    motor = JetsonMotorController(
        left_step_pin=config['left_step_pin'],
        left_dir_pin=config['left_dir_pin'],
        left_enable_pin=config['left_enable_pin'],
        right_step_pin=config['right_step_pin'],
        right_dir_pin=config['right_dir_pin'],
        right_enable_pin=config['right_enable_pin'],
        wheel_diameter=config['wheel_diameter'],
        wheel_base=config['wheel_base'],
        steps_per_rev=config['steps_per_rev'],
        microsteps=config['microsteps'],
        gear_ratio=config['gear_ratio']
    )
    
    if not motor.connect():
        logger.error("Failed to initialize motor controller!")
        imu.disconnect()
        return 1
    
    motor.enable()
    
    # Create calibration object
    calibrator = IMUCalibration(imu, motor)
    
    try:
        # Run calibration
        new_wheel_base = calibrator.calibrate_wheel_base(config['wheel_base'])
        
        # Cleanup
        motor.disable()
        motor.disconnect()
        imu.disconnect()
        
        return 0
        
    except KeyboardInterrupt:
        logger.info("\n\nCalibration interrupted!")
        motor.stop()
        motor.disable()
        motor.disconnect()
        imu.disconnect()
        return 0
    except Exception as e:
        logger.error(f"Calibration failed: {e}", exc_info=True)
        motor.stop()
        motor.disable()
        motor.disconnect()
        imu.disconnect()
        return 1


if __name__ == "__main__":
    sys.exit(main())
