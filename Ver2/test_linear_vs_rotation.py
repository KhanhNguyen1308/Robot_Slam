#!/usr/bin/env python3
"""
Debug Linear vs Rotation Movement
Compare motor behavior in both modes
"""
import sys
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

try:
    from jetson_motor_controller import JetsonMotorController
except ImportError:
    logger.error("Cannot import JetsonMotorController")
    sys.exit(1)


def test_speeds():
    """Test different speeds to debug the 30% issue"""
    logger.info("=" * 70)
    logger.info("LINEAR VS ROTATION DEBUG TEST")
    logger.info("=" * 70)
    logger.info("")
    
    config = {
        'wheel_diameter': 0.06984,  # 69.84mm
        'wheel_base': 0.2845,       # 284.5mm
        'gear_ratio': 38/18         # 2.111
    }
    
    motor = JetsonMotorController(
        left_step_pin=33,
        left_dir_pin=35,
        left_enable_pin=37,
        right_step_pin=32,
        right_dir_pin=36,
        right_enable_pin=38,
        wheel_diameter=config['wheel_diameter'],
        wheel_base=config['wheel_base'],
        steps_per_rev=200,
        microsteps=16,
        gear_ratio=config['gear_ratio']
    )
    
    if not motor.connect():
        logger.error("Failed to initialize motor!")
        return 1
    
    motor.enable()
    
    import math
    
    # Calculate expected speeds
    logger.info("Expected motor speeds:")
    logger.info("")
    
    # Linear test: 0.2 m/s
    linear_speed = 0.2
    steps_per_meter = motor.steps_per_meter
    linear_steps_per_sec = linear_speed * steps_per_meter
    logger.info(f"Linear test (0.2 m/s):")
    logger.info(f"  Steps per meter: {steps_per_meter:.1f}")
    logger.info(f"  Left motor: {linear_steps_per_sec:.1f} steps/s")
    logger.info(f"  Right motor: {linear_steps_per_sec:.1f} steps/s")
    logger.info("")
    
    # Rotation test: 0.5 rad/s
    angular_speed = 0.5
    wheel_arc_velocity = angular_speed * config['wheel_base'] / 2.0
    rotation_steps_per_sec = wheel_arc_velocity * steps_per_meter
    logger.info(f"Rotation test (0.5 rad/s):")
    logger.info(f"  Wheel arc velocity: {wheel_arc_velocity:.4f} m/s")
    logger.info(f"  Left motor: {rotation_steps_per_sec:.1f} steps/s (backward)")
    logger.info(f"  Right motor: {rotation_steps_per_sec:.1f} steps/s (forward)")
    logger.info("")
    
    speed_ratio = linear_steps_per_sec / rotation_steps_per_sec
    logger.info(f"Linear motor speed is {speed_ratio:.1f}x faster than rotation")
    logger.info("")
    
    # Test 1: Short linear movement
    logger.info("=" * 70)
    logger.info("TEST 1: Linear movement - 10 seconds at 0.2 m/s")
    logger.info("Expected distance: 2.0 meters")
    logger.info("=" * 70)
    
    input("Press ENTER to start...")
    
    start = time.time()
    motor.set_velocity(0.2, 0)
    time.sleep(10.0)
    motor.stop()
    elapsed = time.time() - start
    
    logger.info(f"Actual run time: {elapsed:.2f} seconds")
    actual_dist = input("Measure actual distance traveled (meters): ")
    
    try:
        actual = float(actual_dist)
        ratio = actual / 2.0
        logger.info(f"Movement ratio: {ratio:.2f} ({ratio*100:.1f}%)")
        
        if ratio < 0.5:
            logger.warning("⚠️  Robot moved MUCH less than expected!")
            logger.warning("   Possible causes:")
            logger.warning("   - Motor timing issue (Python sleep not accurate)")
            logger.warning("   - Motor stalling (voltage/current too low)")
            logger.warning("   - Mechanical slip")
    except:
        pass
    
    time.sleep(2)
    
    # Test 2: Equivalent rotation
    logger.info("")
    logger.info("=" * 70)
    logger.info("TEST 2: Rotation - equivalent motor speed")
    logger.info("Rotating at slower angular velocity to match linear motor speed")
    logger.info("=" * 70)
    
    # To make rotation use same motor speed as linear:
    # wheel_arc_velocity = 0.2 m/s (same as linear)
    # angular = wheel_arc_velocity / (wheel_base/2)
    test_arc_vel = 0.2
    test_angular = test_arc_vel / (config['wheel_base'] / 2.0)
    test_duration = (2 * math.pi) / test_angular
    test_degrees = 360
    
    logger.info(f"Angular velocity: {test_angular:.3f} rad/s")
    logger.info(f"Duration for 360°: {test_duration:.1f} seconds")
    
    input("Press ENTER to rotate...")
    
    start = time.time()
    motor.set_velocity(0, test_angular)
    time.sleep(test_duration)
    motor.stop()
    elapsed = time.time() - start
    
    logger.info(f"Actual run time: {elapsed:.2f} seconds")
    
    response = input("Did robot complete 360° rotation? (y/n): ")
    
    if response.lower() == 'y':
        logger.info("✓ At SAME motor speed, rotation works but linear doesn't!")
        logger.info("  → Problem is NOT motor speed limit")
        logger.info("  → Problem might be:")
        logger.info("     1. Direction pin issue (different for linear vs rotation)")
        logger.info("     2. Motor wiring/polarity problem")
        logger.info("     3. Software bug in differential drive calculation")
    else:
        logger.info("✗ Both fail at same motor speed")
        logger.info("  → Motor speed limit or timing issue")
    
    motor.disable()
    motor.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(test_speeds())
    except KeyboardInterrupt:
        logger.info("\nTest interrupted")
        sys.exit(0)
