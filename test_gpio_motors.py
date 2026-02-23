"""
Test script for Jetson Nano GPIO motor controller with A4988 drivers
Run this to verify motor connections before running the full system
"""
import time
import sys
from jetson_motor_controller import JetsonMotorController

def test_motors():
    """Test each motor individually"""
    print("=" * 50)
    print("A4988 Motor Controller Test")
    print("=" * 50)
    print()
    
    try:
        # Initialize controller
        print("Initializing motor controller...")
        controller = JetsonMotorController()
        
        if not controller.connect():
            print("❌ Failed to initialize motors")
            return False
        
        print("✓ Motor controller initialized")
        print(f"  Left motor microsteps: 1/{controller.left_motor.microsteps}")
        print(f"  Right motor microsteps: 1/{controller.right_motor.microsteps}")
        print()
        
        # Enable motors
        print("Enabling motors...")
        controller.enable()
        time.sleep(0.5)
        print("✓ Motors enabled")
        print()
        
        # Test left motor forward
        print("Testing LEFT motor FORWARD (2 seconds)...")
        controller.send_velocity(0.1, 0)  # Slow forward
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test left motor backward
        print("Testing LEFT motor BACKWARD (2 seconds)...")
        controller.send_velocity(-0.1, 0)  # Slow backward
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test right motor forward
        print("Testing RIGHT motor FORWARD (2 seconds)...")
        controller.send_velocity(0, 0.5)  # Pure rotation (right wheel forward)
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test right motor backward
        print("Testing RIGHT motor BACKWARD (2 seconds)...")
        controller.send_velocity(0, -0.5)  # Pure rotation (right wheel backward)
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test both motors forward
        print("Testing BOTH motors FORWARD (2 seconds)...")
        controller.send_velocity(0.1, 0)
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test rotation clockwise
        print("Testing ROTATION CW (2 seconds)...")
        controller.send_velocity(0, 1.0)
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        # Test rotation counter-clockwise
        print("Testing ROTATION CCW (2 seconds)...")
        controller.send_velocity(0, -1.0)
        time.sleep(2)
        controller.send_velocity(0, 0)
        time.sleep(1)
        
        print()
        print("=" * 50)
        print("✓ All tests completed successfully!")
        print("=" * 50)
        
        # Cleanup
        controller.disable()
        controller.disconnect()
        
        return True
        
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        controller.send_velocity(0, 0)
        controller.disable()
        controller.disconnect()
        return False
        
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
        try:
            controller.send_velocity(0, 0)
            controller.disable()
            controller.disconnect()
        except:
            pass
        return False


if __name__ == "__main__":
    print("\n⚠ WARNING: Ensure robot is on a stand or wheels can't touch ground!")
    print("Press Enter to continue or Ctrl+C to cancel...")
    try:
        input()
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)
    
    success = test_motors()
    sys.exit(0 if success else 1)
