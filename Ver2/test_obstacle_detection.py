#!/usr/bin/env python3
"""
Test Obstacle Detection System
Demonstrates obstacle avoidance without full robot system
"""
import numpy as np
import cv2
import sys
import time
from obstacle_detector import ObstacleDetector

def create_test_disparity_map(width=640, height=480, obstacle_scenario='front'):
    """
    Create a synthetic disparity map for testing
    
    Args:
        width, height: Image dimensions
        obstacle_scenario: Type of obstacle scenario to simulate
            - 'front': Obstacle directly ahead
            - 'left': Obstacle on left side
            - 'right': Obstacle on right side
            - 'both': Obstacles on both sides
            - 'clear': No obstacles
    
    Returns:
        Simulated disparity map
    """
    # Create empty disparity map
    disparity = np.zeros((height, width), dtype=np.float32)
    
    # Add some background (far away - low disparity)
    disparity[:, :] = np.random.uniform(5, 10, (height, width))
    
    if obstacle_scenario == 'front':
        # Large obstacle in center
        y1, y2 = height//3, 2*height//3
        x1, x2 = width//3, 2*width//3
        # High disparity = close object
        disparity[y1:y2, x1:x2] = np.random.uniform(50, 80, (y2-y1, x2-x1))
        print("Scenario: Obstacle directly ahead")
        
    elif obstacle_scenario == 'left':
        # Obstacle on left side
        y1, y2 = height//3, 2*height//3
        x1, x2 = 0, width//3
        disparity[y1:y2, x1:x2] = np.random.uniform(50, 80, (y2-y1, x2-x1))
        print("Scenario: Obstacle on LEFT side")
        
    elif obstacle_scenario == 'right':
        # Obstacle on right side
        y1, y2 = height//3, 2*height//3
        x1, x2 = 2*width//3, width
        disparity[y1:y2, x1:x2] = np.random.uniform(50, 80, (y2-y1, x2-x1))
        print("Scenario: Obstacle on RIGHT side")
        
    elif obstacle_scenario == 'both':
        # Obstacles on both sides
        y1, y2 = height//3, 2*height//3
        # Left
        x1, x2 = 0, width//4
        disparity[y1:y2, x1:x2] = np.random.uniform(50, 80, (y2-y1, x2-x1))
        # Right
        x1, x2 = 3*width//4, width
        disparity[y1:y2, x1:x2] = np.random.uniform(50, 80, (y2-y1, x2-x1))
        print("Scenario: Obstacles on BOTH sides")
        
    elif obstacle_scenario == 'critical':
        # Very close obstacle (emergency stop)
        y1, y2 = height//2 - 50, height//2 + 50
        x1, x2 = width//2 - 100, width//2 + 100
        disparity[y1:y2, x1:x2] = np.random.uniform(100, 150, (y2-y1, x2-x1))
        print("Scenario: CRITICAL - Very close obstacle!")
        
    else:  # 'clear'
        print("Scenario: Path CLEAR")
    
    return disparity


def test_obstacle_detection():
    """Test obstacle detection with various scenarios"""
    print("=" * 70)
    print("OBSTACLE DETECTION SYSTEM TEST")
    print("=" * 70)
    print()
    
    # Initialize detector
    detector = ObstacleDetector(
        min_safe_distance=0.3,
        critical_distance=0.15,
        detection_width=0.5,
        max_detection_range=2.0
    )
    
    print(f"Detector settings:")
    print(f"  Safe distance: {detector.min_safe_distance}m")
    print(f"  Critical distance: {detector.critical_distance}m")
    print(f"  Detection width: {detector.detection_width}m")
    print()
    
    # Camera parameters (typical for 640x480)
    camera_matrix = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    
    baseline = 0.06  # 60mm baseline
    
    # Test scenarios
    scenarios = ['clear', 'front', 'left', 'right', 'both', 'critical']
    
    # Test with different robot velocities
    test_velocities = [
        (0.2, 0.0, "Moving forward"),
        (0.2, 0.5, "Moving forward + turning left"),
        (0.2, -0.5, "Moving forward + turning right"),
        (0.0, 0.0, "Stopped")
    ]
    
    for scenario in scenarios:
        print("\n" + "=" * 70)
        print(f"SCENARIO: {scenario.upper()}")
        print("=" * 70)
        
        # Create test disparity map
        disparity = create_test_disparity_map(scenario=scenario)
        
        # Test with moving forward
        linear, angular, desc = test_velocities[0]
        
        print(f"\nRobot velocity: {desc} (linear={linear:.2f}m/s, angular={angular:.2f}rad/s)")
        print("-" * 70)
        
        # Run detection
        result = detector.detect_from_disparity(
            disparity, camera_matrix, baseline, (linear, angular)
        )
        
        # Print results
        print(f"\nDetection Results:")
        print(f"  Obstacles found: {len(result['obstacles'])}")
        print(f"  Closest distance: {result['closest_obstacle_distance']:.3f}m" 
              if result['closest_obstacle_distance'] else "  No obstacles in range")
        
        print(f"\nZone Status:")
        for zone_name, status in result['zone_status'].items():
            state = "✗ BLOCKED" if status['has_obstacle'] else "✓ Clear"
            if status.get('critical'):
                state = "⚠ CRITICAL!"
            dist = f"{status['min_distance']:.2f}m" if status['min_distance'] else "---"
            print(f"  {zone_name:15s}: {state:12s}  Distance: {dist}")
        
        action = result['action']
        print(f"\nRecommended Action:")
        print(f"  Severity: {action['severity'].upper()}")
        print(f"  Message: {action['message']}")
        print(f"  Stop Required: {'YES' if action['stop_required'] else 'NO'}")
        print(f"  Recommended velocity: linear={action['recommended_linear']:.2f}m/s, "
              f"angular={action['recommended_angular']:.2f}rad/s")
        
        # Apply safety velocity
        safe_linear, safe_angular = detector.get_safety_velocity(
            linear, angular, result
        )
        
        print(f"\nSafety Controller:")
        print(f"  Desired:  linear={linear:.2f}m/s, angular={angular:.2f}rad/s")
        print(f"  Safe:     linear={safe_linear:.2f}m/s, angular={safe_angular:.2f}rad/s")
        
        if abs(safe_linear - linear) > 0.01 or abs(safe_angular - angular) > 0.01:
            print(f"  ⚠ Velocity MODIFIED for safety!")
        else:
            print(f"  ✓ Velocity unchanged - path clear")
        
        time.sleep(0.5)  # Pause between tests
    
    print("\n" + "=" * 70)
    print("STATISTICS")
    print("=" * 70)
    stats = detector.get_stats()
    print(f"  Obstacles detected: {stats['obstacles_detected']}")
    print(f"  Collision warnings: {stats['collision_warnings']}")
    print(f"  Emergency stops: {stats['emergency_stops']}")
    print()
    print("=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)


def test_visualization():
    """Test obstacle visualization"""
    print("\n" + "=" * 70)
    print("VISUALIZATION TEST")
    print("=" * 70)
    print("\nCreating test visualization (press 'q' to close each window)...")
    
    detector = ObstacleDetector()
    
    camera_matrix = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    
    baseline = 0.06
    
    # Create test scenarios
    scenarios = ['clear', 'front', 'critical']
    
    for scenario in scenarios:
        # Create disparity
        disparity = create_test_disparity_map(scenario=scenario)
        
        # Create a color image for visualization
        # Convert disparity to color image
        disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disparity_colored = cv2.applyColorMap(disparity_normalized.astype(np.uint8), 
                                             cv2.COLORMAP_JET)
        
        # Run detection
        result = detector.detect_from_disparity(
            disparity, camera_matrix, baseline, (0.2, 0.0)
        )
        
        # Add visualization overlay
        vis_image = detector.visualize_obstacles(disparity_colored, result)
        
        # Display
        window_name = f"Obstacle Detection - {scenario.upper()}"
        cv2.imshow(window_name, vis_image)
        print(f"  Showing: {scenario} (press any key to continue)")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)
    
    print("\n✓ Visualization test complete")


if __name__ == "__main__":
    print("\n*** OBSTACLE DETECTION TEST SUITE ***\n")
    
    # Run basic detection tests
    test_obstacle_detection()
    
    # Check if OpenCV is available for visualization
    print("\nTest visualization? (requires display)")
    response = input("Run visualization test? [y/N]: ").strip().lower()
    
    if response == 'y':
        try:
            test_visualization()
        except Exception as e:
            print(f"\n✗ Visualization test failed: {e}")
            print("  (This is normal on headless systems)")
    
    print("\n*** ALL TESTS COMPLETE ***\n")
