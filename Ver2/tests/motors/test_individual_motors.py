#!/usr/bin/env python3
"""
Test Individual Motors
Run each motor separately to check direction and behavior
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


def main():
    logger.info("=" * 70)
    logger.info("INDIVIDUAL MOTOR TEST")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This test runs each motor individually to verify:")
    logger.info("  1. Both motors can run")
    logger.info("  2. Direction is correct")
    logger.info("  3. Speed is consistent")
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
    
    # Test 1: Left motor only (right motor stopped)
    logger.info("=" * 70)
    logger.info("TEST 1: LEFT MOTOR ONLY")
    logger.info("Right motor will be stopped. Robot should turn RIGHT.")
    logger.info("=" * 70)
    
    input("Press ENTER to run left motor at 0.1 m/s for 3 seconds...")
    
    # To run only left motor in current system:
    # Use rotation with negative angular velocity
    # This makes left=backward, right=forward, but we invert right so both forward
    # Actually, let me use direct motor control
    
    logger.info("Running left motor forward...")
    motor.left_motor.set_speed(3000)  # steps/s
    time.sleep(3)
    motor.left_motor.stop()
    
    logger.info("Left motor stopped.")
    logger.info("Did robot turn to the RIGHT? (y/n): ", end='')
    response1 = input()
    
    time.sleep(2)
    
    # Test 2: Right motor only
    logger.info("")
    logger.info("=" * 70)
    logger.info("TEST 2: RIGHT MOTOR ONLY")
    logger.info("Left motor will be stopped. Robot should turn LEFT.")
    logger.info("=" * 70)
    
    input("Press ENTER to run right motor at same speed for 3 seconds...")
    
    logger.info("Running right motor forward...")
    motor.right_motor.set_speed(3000)  # steps/s
    time.sleep(3)
    motor.right_motor.stop()
    
    logger.info("Right motor stopped.")
    logger.info("Did robot turn to the LEFT? (y/n): ", end='')
    response2 = input()
    
    time.sleep(2)
    
    # Test 3: Both motors same direction
    logger.info("")
    logger.info("=" * 70)
    logger.info("TEST 3: BOTH MOTORS FORWARD (same speed)")
    logger.info("Robot should move STRAIGHT FORWARD.")
    logger.info("=" * 70)
    
    input("Press ENTER to run both motors for 5 seconds...")
    
    logger.info("Running both motors at 3000 steps/s...")
    motor.left_motor.set_speed(3000)
    motor.right_motor.set_speed(3000)
    time.sleep(5)
    motor.stop()
    
    logger.info("Motors stopped.")
    dist = input("Measure distance traveled (meters): ")
    
    try:
        actual = float(dist)
        expected_steps = 3000 * 5  # 15000 steps
        expected_dist = expected_steps / motor.steps_per_meter
        
        logger.info("")
        logger.info(f"Expected distance: {expected_dist:.3f}m")
        logger.info(f"Actual distance:   {actual:.3f}m")
        logger.info(f"Ratio: {actual/expected_dist:.2f}")
        
        if actual < expected_dist * 0.5:
            logger.warning("⚠️  Robot moved MUCH less than expected!")
    except:
        pass
    
    time.sleep(2)
    
    # Test 4: Both motors opposite direction
    logger.info("")
    logger.info("=" * 70)
    logger.info("TEST 4: MOTORS OPPOSITE DIRECTION")
    logger.info("Left forward, Right backward → Robot should spin RIGHT")
    logger.info("=" * 70)
    
    input("Press ENTER to test rotation...")
    
    logger.info("Left forward, Right backward for 5 seconds...")
    motor.left_motor.set_speed(2000)   # Forward
    motor.right_motor.set_speed(-2000)  # Backward
    time.sleep(5)
    motor.stop()
    
    logger.info("Motors stopped.")
    logger.info("Did robot rotate? Which direction? (left/right/straight/back): ", end='')
    response3 = input()
    
    # Analysis
    logger.info("")
    logger.info("=" * 70)
    logger.info("ANALYSIS")
    logger.info("=" * 70)
    
    if response3 == "back" or response3 == "backward":
        logger.error("❌ PROBLEM FOUND!")
        logger.error("   Both motors running 'forward' makes robot go BACKWARD!")
        logger.error("   → Motor wiring is INVERTED")
        logger.error("")
        logger.error("Solution: Swap motor wires OR change software inversion")
    elif response3 == "straight":
        logger.error("❌ PROBLEM FOUND!")
        logger.error("   Opposite motor directions make robot go STRAIGHT!")
        logger.error("   → One motor is inverted in hardware")
    elif response3 == "right" or response3 == "left":
        logger.info(f"✓ Robot rotated {response3}")
        logger.info("  Motor directions seem correct")
    
    motor.disable()
    motor.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        logger.info("\nTest interrupted")
        sys.exit(0)
