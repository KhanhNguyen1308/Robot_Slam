#!/usr/bin/env python3
"""
Example Python scripts for robot control
Demonstrates various control patterns and use cases
"""
import requests
import time
import math


class RobotClient:
    """
    Simple Python client for robot control
    """
    
    def __init__(self, host='localhost', port=5000):
        self.base_url = f'http://{host}:{port}/api'
        
    def enable(self):
        """Enable motors"""
        response = requests.post(f'{self.base_url}/enable')
        return response.json()['success']
    
    def disable(self):
        """Disable motors"""
        response = requests.post(f'{self.base_url}/disable')
        return response.json()['success']
    
    def stop(self):
        """Emergency stop"""
        response = requests.post(f'{self.base_url}/stop')
        return response.json()['success']
    
    def set_velocity(self, linear, angular):
        """
        Set robot velocity
        
        Args:
            linear: Linear velocity in m/s (forward positive)
            angular: Angular velocity in rad/s (counter-clockwise positive)
        """
        response = requests.post(
            f'{self.base_url}/velocity',
            json={'linear': linear, 'angular': angular}
        )
        return response.json()['success']
    
    def get_status(self):
        """Get robot status"""
        response = requests.get(f'{self.base_url}/status')
        return response.json()


# ===== EXAMPLES =====

def example_1_basic_movement(robot):
    """Example 1: Basic forward/backward movement"""
    print("Example 1: Basic Movement")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    # Move forward
    print("Moving forward...")
    robot.set_velocity(0.2, 0)
    time.sleep(2)
    
    # Stop
    print("Stopping...")
    robot.set_velocity(0, 0)
    time.sleep(1)
    
    # Move backward
    print("Moving backward...")
    robot.set_velocity(-0.2, 0)
    time.sleep(2)
    
    # Stop
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def example_2_rotation(robot):
    """Example 2: Rotation in place"""
    print("Example 2: Rotation")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    # Rotate 90 degrees (approximately)
    # At 1.0 rad/s, 90 degrees = π/2 radians ≈ 1.57 seconds
    print("Rotating 90 degrees...")
    robot.set_velocity(0, 1.0)
    time.sleep(1.57)
    
    robot.set_velocity(0, 0)
    time.sleep(1)
    
    # Rotate back
    print("Rotating back...")
    robot.set_velocity(0, -1.0)
    time.sleep(1.57)
    
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def example_3_square_path(robot):
    """Example 3: Drive in a square"""
    print("Example 3: Square Path")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    side_length = 1.0  # meters
    speed = 0.2  # m/s
    turn_speed = 1.0  # rad/s
    
    for i in range(4):
        # Drive forward
        print(f"Side {i+1}: Moving forward...")
        robot.set_velocity(speed, 0)
        time.sleep(side_length / speed)
        
        # Stop
        robot.set_velocity(0, 0)
        time.sleep(0.5)
        
        # Turn 90 degrees
        print(f"Side {i+1}: Turning...")
        robot.set_velocity(0, turn_speed)
        time.sleep(math.pi / 2 / turn_speed)
        
        robot.set_velocity(0, 0)
        time.sleep(0.5)
    
    robot.disable()
    print("Done!\n")


def example_4_circle_path(robot):
    """Example 4: Drive in a circle"""
    print("Example 4: Circle Path")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    # For a circle: linear = r * angular
    # Let's make a 1m radius circle
    radius = 1.0  # meters
    linear_speed = 0.2  # m/s
    angular_speed = linear_speed / radius  # rad/s
    
    # Time for one complete circle: 2πr / v
    circle_time = 2 * math.pi * radius / linear_speed
    
    print(f"Driving in a circle (radius={radius}m)...")
    robot.set_velocity(linear_speed, angular_speed)
    time.sleep(circle_time)
    
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def example_5_smooth_acceleration(robot):
    """Example 5: Smooth acceleration and deceleration"""
    print("Example 5: Smooth Acceleration")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    max_speed = 0.3
    accel_time = 2.0  # seconds to reach max speed
    steps = 20
    
    # Accelerate
    print("Accelerating...")
    for i in range(steps + 1):
        speed = (i / steps) * max_speed
        robot.set_velocity(speed, 0)
        time.sleep(accel_time / steps)
    
    # Maintain speed
    print("Cruising...")
    time.sleep(2)
    
    # Decelerate
    print("Decelerating...")
    for i in range(steps, -1, -1):
        speed = (i / steps) * max_speed
        robot.set_velocity(speed, 0)
        time.sleep(accel_time / steps)
    
    robot.disable()
    print("Done!\n")


def example_6_figure_eight(robot):
    """Example 6: Figure-8 pattern"""
    print("Example 6: Figure-8 Pattern")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    radius = 0.5
    linear_speed = 0.2
    
    # First loop (clockwise)
    print("First loop...")
    angular_speed = linear_speed / radius
    circle_time = math.pi * radius / linear_speed  # Half circle
    
    robot.set_velocity(linear_speed, angular_speed)
    time.sleep(circle_time)
    
    # Second loop (counter-clockwise)
    print("Second loop...")
    robot.set_velocity(linear_speed, -angular_speed)
    time.sleep(circle_time)
    
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def example_7_status_monitoring(robot):
    """Example 7: Monitor robot status while moving"""
    print("Example 7: Status Monitoring")
    print("-" * 40)
    
    robot.enable()
    time.sleep(0.5)
    
    robot.set_velocity(0.2, 0)
    
    # Monitor for 5 seconds
    for i in range(10):
        status = robot.get_status()
        print(f"Status: Motors={status['motor_controller']['enabled']}, "
              f"SLAM={status['slam']['tracking']}, "
              f"Commands={status['motor_controller']['commands_sent']}")
        time.sleep(0.5)
    
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def example_8_obstacle_avoidance_pattern(robot):
    """Example 8: Simple obstacle avoidance pattern"""
    print("Example 8: Obstacle Avoidance Pattern")
    print("-" * 40)
    print("This demonstrates a typical obstacle avoidance behavior:")
    print("Move forward → Turn → Move forward → Turn back")
    
    robot.enable()
    time.sleep(0.5)
    
    # Move forward
    print("Moving forward...")
    robot.set_velocity(0.2, 0)
    time.sleep(1)
    
    # "Detect obstacle" - stop
    print("Obstacle detected! Stopping...")
    robot.set_velocity(0, 0)
    time.sleep(0.5)
    
    # Turn to avoid
    print("Turning to avoid...")
    robot.set_velocity(0, 1.5)
    time.sleep(1.0)
    
    robot.set_velocity(0, 0)
    time.sleep(0.5)
    
    # Move forward again
    print("Moving forward...")
    robot.set_velocity(0.2, 0)
    time.sleep(1)
    
    # Turn back to original heading
    print("Returning to original heading...")
    robot.set_velocity(0, -1.5)
    time.sleep(1.0)
    
    robot.set_velocity(0, 0)
    robot.disable()
    print("Done!\n")


def main():
    """Run examples"""
    print("=" * 60)
    print("Robot Control Examples")
    print("=" * 60)
    print()
    
    # Create client
    robot = RobotClient(host='localhost', port=5000)
    
    # Check connection
    try:
        status = robot.get_status()
        print(f"✓ Connected to robot")
        print(f"  Motors: {status['motor_controller']['enabled']}")
        print(f"  SLAM: {status['slam']['tracking']}")
        print()
    except Exception as e:
        print(f"✗ Cannot connect to robot: {e}")
        print("Make sure the robot system is running (python3 main.py)")
        return
    
    # Menu
    examples = {
        '1': ('Basic Movement', example_1_basic_movement),
        '2': ('Rotation', example_2_rotation),
        '3': ('Square Path', example_3_square_path),
        '4': ('Circle Path', example_4_circle_path),
        '5': ('Smooth Acceleration', example_5_smooth_acceleration),
        '6': ('Figure-8', example_6_figure_eight),
        '7': ('Status Monitoring', example_7_status_monitoring),
        '8': ('Obstacle Avoidance Pattern', example_8_obstacle_avoidance_pattern),
    }
    
    while True:
        print("\nAvailable Examples:")
        print("-" * 40)
        for key, (name, _) in examples.items():
            print(f"{key}. {name}")
        print("0. Exit")
        print()
        
        choice = input("Select example (0-8): ").strip()
        
        if choice == '0':
            print("Goodbye!")
            break
        
        if choice in examples:
            name, func = examples[choice]
            print(f"\n{'=' * 60}")
            print(f"Running: {name}")
            print('=' * 60)
            
            try:
                func(robot)
            except KeyboardInterrupt:
                print("\nInterrupted! Stopping robot...")
                robot.stop()
                robot.disable()
            except Exception as e:
                print(f"Error: {e}")
                robot.stop()
                robot.disable()
        else:
            print("Invalid choice!")


if __name__ == '__main__':
    main()
