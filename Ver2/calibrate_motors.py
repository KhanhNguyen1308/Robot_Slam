#!/usr/bin/env python3
"""
Test Motor Calibration - Find correct gear ratio for tracks
Helps calibrate wheel_diameter and gear_ratio for accurate odometry
"""
import sys
import time
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# Import motor controller
try:
    from jetson_motor_controller import JetsonMotorController
except ImportError:
    logger.error("Cannot import JetsonMotorController - make sure you're on Jetson")
    sys.exit(1)


def test_linear_motion():
    """Test linear motion and measure actual distance traveled"""
    logger.info("=" * 70)
    logger.info("LINEAR MOTION CALIBRATION TEST")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This test will move the robot forward to calibrate odometry.")
    logger.info("You will need:")
    logger.info("  - Measuring tape")
    logger.info("  - Mark starting position")
    logger.info("  - At least 2 meters of clear space")
    logger.info("")
    
    # Configuration
    config = {
        'left_step_pin': 33,
        'left_dir_pin': 35,
        'left_enable_pin': 37,
        'right_step_pin': 32,
        'right_dir_pin': 36,
        'right_enable_pin': 38,
        'wheel_diameter': 0.06984,  # 69.84mm (CAD outer diameter, confirmed)
        'wheel_base': 0.2845,       # 284.5mm
        'steps_per_rev': 200,
        'microsteps': 16,
        'gear_ratio': 38/18         # Motor rotations per wheel rotation (2.111)
    }
    
    # Create motor controller
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
        return 1
    
    motor.enable()
    
    # Test parameters
    test_distance = 1.0  # 1 meter
    test_speed = 0.2     # 0.2 m/s
    test_duration = test_distance / test_speed  # 5 seconds
    
    logger.info(f"Current configuration:")
    logger.info(f"  Wheel diameter: {config['wheel_diameter']*1000:.1f}mm")
    logger.info(f"  Wheel base: {config['wheel_base']*1000:.1f}mm")
    logger.info(f"  Gear ratio: {config['gear_ratio']:.3f}")
    logger.info("")
    logger.info(f"Test plan: Move at {test_speed}m/s for {test_duration:.1f}s = {test_distance}m")
    logger.info("")
    
    input(f"Press ENTER to move robot forward {test_distance}m...")
    
    logger.info(f"Moving forward at {test_speed}m/s for {test_duration:.1f}s...")
    motor.set_velocity(test_speed, 0)  # Linear forward, no rotation
    time.sleep(test_duration)
    motor.stop()
    
    logger.info("")
    logger.info("Robot stopped!")
    logger.info("")
    logger.info("Measure the ACTUAL distance traveled with measuring tape.")
    
    actual_distance_str = input("Enter actual distance traveled (in meters): ")
    try:
        actual_distance = float(actual_distance_str)
    except:
        logger.error("Invalid input!")
        motor.disable()
        motor.disconnect()
        return 1
    
    # Calculate correction factor
    correction = actual_distance / test_distance
    new_gear_ratio = config['gear_ratio'] / correction
    
    logger.info("")
    logger.info("=" * 70)
    logger.info("CALIBRATION RESULT:")
    logger.info("=" * 70)
    logger.info(f"Commanded distance: {test_distance:.3f}m")
    logger.info(f"Actual distance:    {actual_distance:.3f}m")
    logger.info(f"Error:              {(actual_distance - test_distance)*100:.1f}cm ({((correction-1)*100):.1f}%)")
    logger.info("")
    logger.info(f"Current gear_ratio: {config['gear_ratio']:.4f}")
    logger.info(f"New gear_ratio:     {new_gear_ratio:.4f}")
    logger.info("")
    logger.info("Update main.py config with:")
    logger.info(f"  'gear_ratio': {new_gear_ratio:.4f},")
    logger.info("=" * 70)
    
    motor.disable()
    motor.disconnect()
    
    return 0


def test_rotation():
    """Test rotational motion to calibrate wheel_base"""
    logger.info("=" * 70)
    logger.info("ROTATION CALIBRATION TEST")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This test will rotate the robot 360° to calibrate wheel_base.")
    logger.info("You will need:")
    logger.info("  - Marker or tape to mark starting orientation")
    logger.info("  - At least 1 meter of clear space around robot")
    logger.info("")
    
    # Configuration
    config = {
        'left_step_pin': 33,
        'left_dir_pin': 35,
        'left_enable_pin': 37,
        'right_step_pin': 32,
        'right_dir_pin': 36,
        'right_enable_pin': 38,
        'wheel_diameter': 0.0291,  # 29.1mm (sprocket 20 teeth, calibrated)
        'wheel_base': 0.2845,      # 284.5mm
        'steps_per_rev': 200,
        'microsteps': 16,
        'gear_ratio': 38/18      # Default gear ratio
    }
    
    gear_ratio_str = input("Enter calibrated gear_ratio (or press ENTER for 38/18): ").strip()
    if gear_ratio_str:
        # Support fraction format like "18/38" or decimal like "0.474"
        if '/' in gear_ratio_str:
            parts = gear_ratio_str.split('/')
            config['gear_ratio'] = float(parts[0]) / float(parts[1])
        else:
            config['gear_ratio'] = float(gear_ratio_str)
    else:
        config['gear_ratio'] = 38/18 # Default to actual gear ratio (2.111)
    
    # Create motor controller
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
        return 1
    
    motor.enable()
    
    import math
    test_angle = 2 * math.pi  # 360 degrees (radians)
    test_angular_speed = 0.5  # 0.5 rad/s (~28.6 deg/s)
    test_duration = test_angle / test_angular_speed  # Time to complete 360°
    
    logger.info(f"Current configuration:")
    logger.info(f"  Wheel base: {config['wheel_base']*1000:.1f}mm")
    logger.info(f"  Gear ratio: {config['gear_ratio']:.4f}")
    logger.info("")
    logger.info(f"Test plan: Rotate at {test_angular_speed}rad/s for {test_duration:.1f}s = 360°")
    logger.info("")
    
    input("Mark robot's current orientation, then press ENTER to rotate 360°...")
    
    logger.info(f"Rotating 360° at {test_angular_speed}rad/s for {test_duration:.1f}s...")
    motor.set_velocity(0, test_angular_speed)  # No linear, rotate only
    time.sleep(test_duration)
    motor.stop()
    
    logger.info("")
    logger.info("Robot stopped!")
    logger.info("")
    logger.info("Did the robot return to the SAME orientation? (within ~5°)")
    
    response = input("(y)es / (n)o / (o)vershot / (u)ndershot: ").lower()
    
    logger.info("")
    
    if response == 'y':
        logger.info("✓ Wheel base is correctly calibrated!")
        logger.info(f"  Keep wheel_base: {config['wheel_base']:.4f}m ({config['wheel_base']*1000:.1f}mm)")
    elif response == 'n':
        # Ask for actual angle
        logger.info("")
        logger.info("For accurate calibration, use the IMU-based tool:")
        logger.info("  python3 calibrate_with_imu.py")
        logger.info("")
        logger.info("Or manually calculate:")
        actual_angle_str = input("What was the actual rotation angle in degrees? ")
        try:
            actual_angle = float(actual_angle_str)
            ratio = actual_angle / 360.0
            new_wheel_base = config['wheel_base'] * ratio
            
            logger.info("")
            logger.info("=" * 70)
            logger.info("CALIBRATION RESULT:")
            logger.info("=" * 70)
            logger.info(f"Commanded:          360°")
            logger.info(f"Actual:             {actual_angle}°")
            logger.info(f"Ratio:              {ratio:.3f}")
            logger.info(f"Current wheel_base: {config['wheel_base']*1000:.1f}mm")
            logger.info(f"New wheel_base:     {new_wheel_base*1000:.1f}mm")
            logger.info("")
            logger.info("⚠️  WARNING: New value seems unusual!")
            if new_wheel_base < 0.05:
                logger.info("   Value too small - there may be another issue.")
                logger.info("   Check: gear_ratio, motor wiring, microstepping")
            logger.info("")
            logger.info("Update main.py config with:")
            logger.info(f"  'wheel_base': {new_wheel_base:.4f},  # {new_wheel_base*1000:.1f}mm")
            logger.info("")
            logger.info("RECOMMENDED: Use IMU calibration for accurate results:")
            logger.info("  python3 calibrate_with_imu.py")
            logger.info("=" * 70)
        except:
            logger.error("Invalid input!")
    elif response == 'o':
        degrees_str = input("By how many degrees did it overshoot? ")
        try:
            overshoot = float(degrees_str)
            correction = (360 - overshoot) / 360
            new_wheel_base = config['wheel_base'] * correction
            
            logger.info("")
            logger.info("=" * 70)
            logger.info("CALIBRATION RESULT:")
            logger.info("=" * 70)
            logger.info(f"Overshot by:        {overshoot}°")
            logger.info(f"Current wheel_base: {config['wheel_base']*1000:.1f}mm")
            logger.info(f"New wheel_base:     {new_wheel_base*1000:.1f}mm")
            logger.info("")
            logger.info("Update main.py config with:")
            logger.info(f"  'wheel_base': {new_wheel_base:.4f},  # {new_wheel_base*1000:.1f}mm")
            logger.info("=" * 70)
        except:
            logger.error("Invalid input!")
    elif response == 'u':
        degrees_str = input("By how many degrees did it undershoot? ")
        try:
            undershoot = float(degrees_str)
            correction = (360 + undershoot) / 360
            new_wheel_base = config['wheel_base'] * correction
            
            logger.info("")
            logger.info("=" * 70)
            logger.info("CALIBRATION RESULT:")
            logger.info("=" * 70)
            logger.info(f"Undershot by:       {undershoot}°")
            logger.info(f"Current wheel_base: {config['wheel_base']*1000:.1f}mm")
            logger.info(f"New wheel_base:     {new_wheel_base*1000:.1f}mm")
            logger.info("")
            logger.info("Update main.py config with:")
            logger.info(f"  'wheel_base': {new_wheel_base:.4f},  # {new_wheel_base*1000:.1f}mm")
            logger.info("=" * 70)
        except:
            logger.error("Invalid input!")
    
    motor.disable()
    motor.disconnect()
    
    return 0


def main():
    """Main calibration workflow"""
    logger.info("=" * 70)
    logger.info("ROBOT MOTOR CALIBRATION TOOL")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This tool helps calibrate:")
    logger.info("  1. gear_ratio - for accurate linear distance")
    logger.info("  2. wheel_base - for accurate rotation angle")
    logger.info("")
    logger.info("⚠️  WARNING: Make sure robot has clear space to move!")
    logger.info("")
    
    print("Select test:")
    print("  1. Linear motion (calibrate gear_ratio)")
    print("  2. Rotation (calibrate wheel_base)")
    print("  3. Both (recommended)")
    
    choice = input("\nChoice (1/2/3): ")
    
    if choice == '1':
        return test_linear_motion()
    elif choice == '2':
        return test_rotation()
    elif choice == '3':
        logger.info("\n=== STEP 1: Linear Motion Calibration ===\n")
        ret = test_linear_motion()
        if ret != 0:
            return ret
        
        logger.info("\n\n=== STEP 2: Rotation Calibration ===\n")
        return test_rotation()
    else:
        logger.error("Invalid choice!")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        logger.info("\n\nCalibration interrupted by user")
        sys.exit(0)
