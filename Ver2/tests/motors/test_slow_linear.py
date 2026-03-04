#!/usr/bin/env python3
"""
Test Linear Motion at Different Speeds
Find the maximum speed the motor can handle
"""
import sys
import os
# Allow running from any working directory — project root is 2 levels up
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

try:
    from jetson_motor_controller import JetsonMotorController
except ImportError:
    logger.error("Cannot import JetsonMotorController")
    sys.exit(1)


def test_slow_linear():
    """Test linear motion at slow speed (same as rotation)"""
    logger.info("=" * 70)
    logger.info("SLOW LINEAR MOTION TEST")
    logger.info("=" * 70)
    logger.info("")
    logger.info("Testing at SAME motor speed as rotation test")
    logger.info("If this works, problem is motor speed limit")
    logger.info("")
    
    motor = JetsonMotorController(
        left_step_pin=33,
        left_dir_pin=35,
        left_enable_pin=37,
        right_step_pin=32,
        right_dir_pin=36,
        right_enable_pin=38,
        wheel_diameter=0.06984,
        wheel_base=0.2845,
        steps_per_rev=200,
        microsteps=16,
        gear_ratio=38/18
    )
    
    if not motor.connect():
        logger.error("Failed to initialize!")
        return 1
    
    motor.enable()
    
    # Calculate rotation wheel speed
    rotation_angular = 0.5  # rad/s (from rotation test)
    rotation_wheel_speed = rotation_angular * (0.2845 / 2.0)
    
    logger.info(f"Rotation test uses wheel speed: {rotation_wheel_speed:.4f} m/s")
    logger.info("")
    
    # Test speeds
    test_speeds = [
        (rotation_wheel_speed, "Same as rotation"),
        (0.1, "Medium slow (0.1 m/s)"),
        (0.15, "Medium (0.15 m/s)"),
        (0.2, "Original test (0.2 m/s)")
    ]
    
    for speed, description in test_speeds:
        logger.info("=" * 70)
        logger.info(f"TEST: {description}")
        logger.info(f"Speed: {speed:.4f} m/s")
        
        test_distance = 1.0
        test_duration = test_distance / speed
        
        logger.info(f"Duration: {test_duration:.1f} seconds for 1.0m")
        logger.info("=" * 70)
        
        response = input("Run this test? (y/n/q to quit): ").lower()
        
        if response == 'q':
            break
        elif response != 'y':
            continue
        
        logger.info(f"Running at {speed:.4f} m/s for {test_duration:.1f} seconds...")
        logger.info("Watch if robot runs continuously!")
        
        start_time = time.time()
        motor.set_velocity(speed, 0)
        time.sleep(test_duration)
        motor.stop()
        actual_time = time.time() - start_time
        
        logger.info(f"Done! Actual duration: {actual_time:.2f}s")
        logger.info("")
        
        actual_dist = input("Measure actual distance (meters): ")
        try:
            dist = float(actual_dist)
            ratio = dist / test_distance
            
            logger.info("")
            logger.info(f"Expected: {test_distance:.2f}m")
            logger.info(f"Actual:   {dist:.2f}m")
            logger.info(f"Ratio:    {ratio:.2f} ({ratio*100:.1f}%)")
            
            if ratio > 0.9:
                logger.info("✓ SUCCESS at this speed!")
                logger.info(f"  Maximum working speed: {speed:.4f} m/s")
                
                # Calculate max steps/s
                steps_per_sec = speed * motor.steps_per_meter
                logger.info(f"  Maximum motor speed: {steps_per_sec:.1f} steps/s")
                logger.info("")
                logger.info("Update main.py config with:")
                logger.info(f"  'max_linear_speed': {speed:.2f},  # m/s")
                break
            else:
                logger.warning(f"✗ Still only {ratio*100:.1f}% at this speed")
        except:
            pass
        
        logger.info("")
        time.sleep(2)
    
    motor.disable()
    motor.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(test_slow_linear())
    except KeyboardInterrupt:
        logger.info("\nTest interrupted")
        sys.exit(0)
