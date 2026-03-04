#!/usr/bin/env python3
"""
Debug Motor Movement - Count actual steps and measure distance
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


def debug_linear_motion():
    """Test and measure actual motor behavior"""
    logger.info("=" * 70)
    logger.info("MOTOR MOVEMENT DEBUG")
    logger.info("=" * 70)
    logger.info("")
    
    # Test configurations
    configs = [
        {
            'name': 'Current (38/18)',
            'wheel_diameter': 0.066,
            'gear_ratio': 38/18
        },
        {
            'name': 'Test 3x (114/18)', 
            'wheel_diameter': 0.066,
            'gear_ratio': 114/18  # 3x current
        },
        {
            'name': 'Test smaller wheel (20mm)',
            'wheel_diameter': 0.020,
            'gear_ratio': 38/18
        }
    ]
    
    logger.info("Available test configurations:")
    for i, cfg in enumerate(configs, 1):
        logger.info(f"  {i}. {cfg['name']}")
        logger.info(f"     wheel_diameter={cfg['wheel_diameter']*1000:.1f}mm, gear_ratio={cfg['gear_ratio']:.3f}")
    
    choice = input("\nSelect config to test (1-3): ")
    try:
        config = configs[int(choice) - 1]
    except:
        logger.error("Invalid choice")
        return 1
    
    logger.info(f"\nTesting with: {config['name']}")
    logger.info("")
    
    # Create motor controller
    motor = JetsonMotorController(
        left_step_pin=33,
        left_dir_pin=35,
        left_enable_pin=37,
        right_step_pin=32,
        right_dir_pin=36,
        right_enable_pin=38,
        wheel_diameter=config['wheel_diameter'],
        wheel_base=0.2845,
        steps_per_rev=200,
        microsteps=16,
        gear_ratio=config['gear_ratio']
    )
    
    if not motor.connect():
        logger.error("Failed to initialize motor!")
        return 1
    
    motor.enable()
    
    # Calculate expected values
    circumference = 3.14159 * config['wheel_diameter']
    steps_per_rev = 200 * 16  # 3200 with 1/16 microstepping
    steps_per_meter = (steps_per_rev * config['gear_ratio']) / circumference
    
    logger.info(f"Calculated parameters:")
    logger.info(f"  Wheel circumference: {circumference*1000:.2f}mm")
    logger.info(f"  Steps per revolution: {steps_per_rev}")
    logger.info(f"  Steps per meter: {steps_per_meter:.1f}")
    logger.info("")
    
    # Test different speeds and durations
    test_distance = 1.0  # meters
    test_speed = 0.2     # m/s
    test_duration = test_distance / test_speed
    
    logger.info(f"Test: Command {test_speed}m/s for {test_duration:.1f}s = {test_distance}m")
    logger.info("")
    
    input("Press ENTER to start movement...")
    
    logger.info(f"Moving at {test_speed}m/s for {test_duration:.1f}s...")
    start_time = time.time()
    motor.set_velocity(test_speed, 0)
    time.sleep(test_duration)
    motor.stop()
    actual_duration = time.time() - start_time
    
    logger.info("")
    logger.info(f"Movement complete (actual duration: {actual_duration:.2f}s)")
    logger.info("")
    logger.info("Measure the ACTUAL distance traveled with measuring tape.")
    
    actual_distance_str = input("Enter actual distance in meters: ")
    try:
        actual_distance = float(actual_distance_str)
    except:
        logger.error("Invalid input!")
        motor.disable()
        motor.disconnect()
        return 1
    
    # Calculate what's really happening
    ratio = actual_distance / test_distance
    
    logger.info("")
    logger.info("=" * 70)
    logger.info("ANALYSIS")
    logger.info("=" * 70)
    logger.info(f"Commanded: {test_distance:.3f}m")
    logger.info(f"Actual:    {actual_distance:.3f}m")
    logger.info(f"Ratio:     {ratio:.3f} ({ratio*100:.1f}%)")
    logger.info("")
    
    if ratio < 0.9:
        # Robot moved less than commanded
        logger.info("Robot moved LESS than commanded. Possible causes:")
        logger.info("")
        
        # Calculate corrections
        new_gear_ratio = config['gear_ratio'] / ratio
        new_wheel_diameter = config['wheel_diameter'] * ratio
        
        logger.info("Option 1: Increase gear_ratio")
        logger.info(f"  Current: {config['gear_ratio']:.4f}")
        logger.info(f"  New:     {new_gear_ratio:.4f}")
        logger.info(f"  Change:  {(new_gear_ratio/config['gear_ratio']):.2f}x")
        logger.info("")
        
        logger.info("Option 2: Decrease wheel_diameter")
        logger.info(f"  Current: {config['wheel_diameter']*1000:.1f}mm")
        logger.info(f"  New:     {new_wheel_diameter*1000:.1f}mm")
        logger.info("")
        
        # Analyze gear ratio
        if new_gear_ratio > 10:
            logger.info("⚠️  WARNING: New gear_ratio > 10 is very unusual!")
            logger.info("   This suggests there may be MULTIPLE gear stages.")
            logger.info("")
            logger.info("   Please count ALL gears in the drive train:")
            logger.info("   - Motor shaft: ? teeth")
            logger.info("   - First intermediate: ? teeth")
            logger.info("   - Second intermediate: ? teeth")  
            logger.info("   - Final sprocket: ? teeth")
            logger.info("")
            
        # Analyze wheel diameter
        if new_wheel_diameter < 0.03:
            logger.info("⚠️  WARNING: New wheel_diameter < 30mm is very small!")
            logger.info("   Please MEASURE the actual sprocket:")
            logger.info("   - Pitch diameter (at chain engagement)")
            logger.info("   - Or measure circumference and divide by π")
            logger.info("")
    
    elif ratio > 1.1:
        logger.info("Robot moved MORE than commanded.")
        logger.info("Gear ratio or wheel diameter may be too large.")
    else:
        logger.info("✓ Movement is accurate within 10%!")
    
    logger.info("=" * 70)
    
    motor.disable()
    motor.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(debug_linear_motion())
    except KeyboardInterrupt:
        logger.info("\nTest interrupted")
        sys.exit(0)
