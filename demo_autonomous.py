#!/usr/bin/env python3
"""
Autonomous Navigation Demo
Demonstrates obstacle avoidance and autonomous navigation
"""
import requests
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AutonomousRobot:
    """Autonomous robot controller with obstacle avoidance"""
    
    def __init__(self, host='localhost', port=5000):
        self.base_url = f'http://{host}:{port}/api'
        
    def enable_motors(self):
        """Enable motors"""
        response = requests.post(f'{self.base_url}/enable')
        return response.json()['success']
    
    def disable_motors(self):
        """Disable motors"""
        response = requests.post(f'{self.base_url}/disable')
        return response.json()['success']
    
    def set_autonomous(self, enable=True):
        """Enable/disable autonomous mode"""
        response = requests.post(
            f'{self.base_url}/autonomous',
            json={'enable': enable}
        )
        return response.json()['success']
    
    def get_status(self):
        """Get full status"""
        response = requests.get(f'{self.base_url}/status')
        return response.json()
    
    def get_obstacles(self):
        """Get obstacle information"""
        response = requests.get(f'{self.base_url}/obstacles')
        return response.json()
    
    def get_imu(self):
        """Get IMU data"""
        response = requests.get(f'{self.base_url}/imu')
        return response.json()
    
    def set_velocity(self, linear, angular):
        """Manual velocity control"""
        response = requests.post(
            f'{self.base_url}/velocity',
            json={'linear': linear, 'angular': angular}
        )
        return response.json()['success']


def demo_obstacle_monitoring(robot, duration=10):
    """Demo 1: Monitor obstacles while moving"""
    print("\n" + "=" * 60)
    print("DEMO 1: Obstacle Monitoring")
    print("=" * 60)
    print("Moving forward while monitoring obstacles...")
    
    robot.enable_motors()
    time.sleep(0.5)
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Move forward slowly
        robot.set_velocity(0.2, 0)
        
        # Get obstacle info
        obstacles = robot.get_obstacles()
        zones = obstacles['danger_zones']
        
        print(f"\rZones: L={zones['left']:8s} C={zones['center']:8s} R={zones['right']:8s} | "
              f"Obstacles: {len(obstacles['obstacles'])}", end='')
        
        # Emergency stop if center is in danger
        if zones['center'] == 'danger':
            print("\n⚠ OBSTACLE AHEAD - STOPPING")
            robot.set_velocity(0, 0)
            time.sleep(1)
        
        time.sleep(0.1)
    
    robot.set_velocity(0, 0)
    robot.disable_motors()
    print("\n✓ Demo 1 complete\n")


def demo_autonomous_navigation(robot, duration=30):
    """Demo 2: Fully autonomous navigation"""
    print("\n" + "=" * 60)
    print("DEMO 2: Autonomous Navigation")
    print("=" * 60)
    print("Enabling autonomous obstacle avoidance...")
    
    robot.enable_motors()
    time.sleep(0.5)
    
    # Enable autonomous mode
    robot.set_autonomous(True)
    
    start_time = time.time()
    
    print("Robot is now navigating autonomously!")
    print("Press Ctrl+C to stop\n")
    
    try:
        while time.time() - start_time < duration:
            # Just monitor status
            status = robot.get_status()
            obstacles = status['obstacles']
            
            print(f"\rAutonomous: {status['autonomous_mode']} | "
                  f"Obstacles: {len(obstacles['obstacles'])} | "
                  f"Safe Dir: {obstacles['safe_direction']:8s} | "
                  f"Motors: {status['motor_controller']['enabled']}", end='')
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nStopping autonomous mode...")
    
    # Disable autonomous mode
    robot.set_autonomous(False)
    robot.set_velocity(0, 0)
    robot.disable_motors()
    print("\n✓ Demo 2 complete\n")


def demo_imu_monitoring(robot, duration=10):
    """Demo 3: Monitor IMU during movement"""
    print("\n" + "=" * 60)
    print("DEMO 3: IMU Monitoring")
    print("=" * 60)
    print("Monitoring tilt and acceleration...")
    
    robot.enable_motors()
    time.sleep(0.5)
    
    start_time = time.time()
    
    # Move in a pattern
    pattern = [
        (0.2, 0, 2),    # Forward
        (0, 1.0, 1.5),  # Turn left
        (0.2, 0, 2),    # Forward
        (0, -1.0, 1.5), # Turn right
    ]
    
    pattern_idx = 0
    step_start = time.time()
    
    while time.time() - start_time < duration:
        # Execute pattern
        if pattern_idx < len(pattern):
            linear, angular, step_duration = pattern[pattern_idx]
            
            if time.time() - step_start > step_duration:
                pattern_idx += 1
                step_start = time.time()
            else:
                robot.set_velocity(linear, angular)
        
        # Get IMU data
        imu = robot.get_imu()
        
        if imu['connected']:
            print(f"\rPitch: {imu['pitch']:6.1f}° | "
                  f"Roll: {imu['roll']:6.1f}° | "
                  f"Accel: {imu['magnitude']:.2f}g | "
                  f"Collision: {'YES' if imu['collision'] else 'NO '}", end='')
        else:
            print("\rIMU not connected", end='')
        
        time.sleep(0.1)
    
    robot.set_velocity(0, 0)
    robot.disable_motors()
    print("\n✓ Demo 3 complete\n")


def demo_obstacle_avoidance_pattern(robot):
    """Demo 4: Programmed obstacle avoidance pattern"""
    print("\n" + "=" * 60)
    print("DEMO 4: Obstacle Avoidance Pattern")
    print("=" * 60)
    print("Manual control with obstacle awareness...")
    
    robot.enable_motors()
    time.sleep(0.5)
    
    for i in range(10):  # 10 iterations
        print(f"\nIteration {i+1}/10")
        
        # Get obstacles
        obstacles = robot.get_obstacles()
        zones = obstacles['danger_zones']
        safe_dir = obstacles['safe_direction']
        
        print(f"  Zones: {zones}")
        print(f"  Safe direction: {safe_dir}")
        
        # Decide action based on safe direction
        if safe_dir == 'forward':
            print("  → Moving forward")
            robot.set_velocity(0.3, 0)
            time.sleep(1)
        elif safe_dir == 'left':
            print("  → Turning left")
            robot.set_velocity(0, 1.0)
            time.sleep(1.0)
        elif safe_dir == 'right':
            print("  → Turning right")
            robot.set_velocity(0, -1.0)
            time.sleep(1.0)
        elif safe_dir == 'backward':
            print("  → Moving backward")
            robot.set_velocity(-0.2, 0)
            time.sleep(1)
        
        robot.set_velocity(0, 0)
        time.sleep(0.5)
    
    robot.disable_motors()
    print("\n✓ Demo 4 complete\n")


def main():
    print("=" * 60)
    print("AUTONOMOUS NAVIGATION DEMOS")
    print("=" * 60)
    
    # Connect to robot
    robot = AutonomousRobot()
    
    # Check connection
    try:
        status = robot.get_status()
        print(f"\n✓ Connected to robot")
        print(f"  Motors: {status['motor_controller']['enabled']}")
        print(f"  IMU: {'Connected' if status['imu']['connected'] else 'Not available'}")
        print(f"  Obstacles detected: {len(status['obstacles']['obstacles'])}")
    except Exception as e:
        print(f"\n✗ Cannot connect to robot: {e}")
        print("Make sure the robot system is running (python3 main.py)")
        return
    
    # Menu
    demos = {
        '1': ('Obstacle Monitoring', lambda: demo_obstacle_monitoring(robot, 10)),
        '2': ('Autonomous Navigation', lambda: demo_autonomous_navigation(robot, 30)),
        '3': ('IMU Monitoring', lambda: demo_imu_monitoring(robot, 10)),
        '4': ('Obstacle Avoidance Pattern', lambda: demo_obstacle_avoidance_pattern(robot)),
    }
    
    while True:
        print("\n" + "=" * 60)
        print("Available Demos:")
        print("-" * 60)
        for key, (name, _) in demos.items():
            print(f"{key}. {name}")
        print("0. Exit")
        print()
        
        choice = input("Select demo (0-4): ").strip()
        
        if choice == '0':
            print("Goodbye!")
            break
        
        if choice in demos:
            name, func = demos[choice]
            
            try:
                func()
            except KeyboardInterrupt:
                print("\n\nDemo interrupted!")
                robot.set_velocity(0, 0)
                robot.disable_motors()
                robot.set_autonomous(False)
            except Exception as e:
                print(f"\nError during demo: {e}")
                robot.set_velocity(0, 0)
                robot.disable_motors()
                robot.set_autonomous(False)
        else:
            print("Invalid choice!")


if __name__ == '__main__':
    main()