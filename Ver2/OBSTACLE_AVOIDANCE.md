# OBSTACLE AVOIDANCE SYSTEM

## Tổng Quan

Hệ thống obstacle avoidance (tránh vật cản) real-time cho autonomous mapping robot sử dụng stereo depth maps để phát hiện và tránh va chạm.

## Các Component

### 1. **ObstacleDetector** (`obstacle_detector.py`)

Module chính để phát hiện vật cản từ stereo disparity maps.

**Tính năng:**
- ✅ Real-time obstacle detection từ depth maps
- ✅ Multi-zone detection (front center, left, right, sides)
- ✅ Phân loại mức độ nguy hiểm (critical, warning, caution)
- ✅ Tự động tính toán safe velocities
- ✅ Obstacle tracking và history
- ✅ Visualization support

**Cấu hình:**
```python
detector = ObstacleDetector(
    min_safe_distance=0.3,      # meters - khoảng cách an toàn tối thiểu
    critical_distance=0.15,     # meters - khoảng cách nguy hiểm (dừng khẩn cấp)
    detection_width=0.5,        # meters - độ rộng vùng phát hiện
    max_detection_range=2.0     # meters - khoảng cách phát hiện tối đa
)
```

**Detection Zones:**
```
          [FRONT LEFT]  [FRONT CENTER]  [FRONT RIGHT]
                \            |            /
                 \           |           /
                  \          |          /
    [LEFT] -------- [  ROBOT  ] -------- [RIGHT]
```

### 2. **AutonomousMapper Integration**

Tích hợp obstacle avoidance vào autonomous navigation.

**Tính năng:**
- ✅ Tự động điều chỉnh velocity khi phát hiện vật cản
- ✅ Smart obstacle avoidance trong exploration
- ✅ Real-time safety checking
- ✅ Statistics tracking

**Sử dụng:**
```python
# Autonomous mapper tự động sử dụng obstacle detector
mapper = AutonomousMapper(
    motor_controller=motor,
    slam_system=slam,
    occupancy_grid=grid,
    obstacle_detector=detector,  # ← Tự động enable obstacle avoidance
    max_linear_speed=0.2,
    max_angular_speed=1.0
)

# Enable/disable obstacle avoidance
mapper.obstacle_avoidance_enabled = True  # Default
```

### 3. **SafetyVelocityController** (`safety_controller.py`)

Wrapper cho manual control với obstacle avoidance.

**Tính năng:**
- ✅ Real-time safety checking cho teleop
- ✅ Tự động block/modify unsafe commands
- ✅ Statistics tracking
- ✅ Can enable/disable

**Sử dụng:**
```python
from safety_controller import SafetyVelocityController

# Wrap motor controller
safe_motor = SafetyVelocityController(
    motor_controller=motor,
    obstacle_detector=detector,
    enable_by_default=True
)

# Update obstacle detection
safe_motor.update_obstacle_detection(detection_result)

# Send velocity (automatically safe)
safe_motor.send_velocity(linear=0.3, angular=0.0)

# Force unsafe command (bypass safety)
safe_motor.send_velocity(linear=0.3, angular=0.0, force=True)

# Get obstacle status
status = safe_motor.get_obstacle_status()
print(status)
# {
#   'has_obstacles': True,
#   'severity': 'warning',
#   'message': 'Obstacle ahead - turning left',
#   'closest_distance': 0.25,
#   'stop_required': False
# }
```

## Workflow

### 1. Detection Flow
```
Stereo Camera
     ↓
Disparity Map
     ↓
ObstacleDetector.detect_from_disparity()
     ↓
Detection Result
{
  'obstacles': [...],
  'zone_status': {...},
  'action': {
    'stop_required': bool,
    'recommended_linear': float,
    'recommended_angular': float,
    'severity': 'critical'|'warning'|'caution'|'none'
  }
}
```

### 2. Navigation Flow
```
Desired Velocity (from path planner)
     ↓
ObstacleDetector.get_safety_velocity()
     ↓
Safe Velocity (adjusted for obstacles)
     ↓
Motor Controller
```

## Configuration trong main.py

```python
config = {
    'obstacle_detection': {
        'min_safe_distance': 0.3,    # Khoảng cách an toàn (m)
        'critical_distance': 0.15,   # Khoảng cách nguy hiểm (m)
        'detection_width': 0.5,      # Độ rộng robot + margin (m)
        'max_range': 2.0             # Phạm vi phát hiện tối đa (m)
    }
}
```

## Testing

### Test Script
```bash
cd /home/jetson/Robot_Slam/Ver2
python3 test_obstacle_detection.py
```

**Test scenarios:**
- ✓ Clear path
- ✓ Obstacle ahead (front center)
- ✓ Obstacle on left
- ✓ Obstacle on right
- ✓ Obstacles on both sides
- ✓ Critical obstacle (emergency stop)

### Kết quả mẫu:
```
SCENARIO: FRONT
---------------------------------
Obstacles found: 1
Closest distance: 0.234m

Zone Status:
  front_center  : ✗ BLOCKED      Distance: 0.23m
  front_left    : ✓ Clear        Distance: ---
  front_right   : ✓ Clear        Distance: ---

Recommended Action:
  Severity: WARNING
  Message: Obstacle ahead - turning right
  Stop Required: NO
  Recommended velocity: linear=0.06m/s, angular=-0.50rad/s
```

## Integration với Main Application

Obstacle detection được tự động tích hợp vào main loop:

```python
# main.py _main_loop()

# 1. Get frames from stereo camera
left, right, disparity = self.stereo_camera.get_latest_frames()

# 2. Run obstacle detection
obstacle_result = self.autonomous_mapper.update_obstacle_detection(
    disparity, camera_matrix, baseline
)

# 3. Navigation tự động sử dụng safe velocities
self.autonomous_mapper._navigate_to_goal(goal)
# ↑ Internally calls obstacle_detector.get_safety_velocity()
```

## Performance

**Typical performance trên Jetson Nano:**
- Obstacle detection: ~20-30 FPS @ 640x480
- Latency: <50ms từ disparity → safe velocity
- Memory: ~50MB cho detector + history

**Optimization tips:**
- Giảm resolution nếu cần FPS cao hơn (320x240)
- Adjust obstacle_history_size (default=5 frames)
- Sử dụng GPU stereo matching nếu có thể

## Statistics & Monitoring

### Obstacle Detector Stats
```python
stats = detector.get_stats()
# {
#   'obstacles_detected': 142,
#   'collision_warnings': 23,
#   'emergency_stops': 2,
#   'history_size': 5
# }
```

### Mapper Stats
```python
stats = mapper.get_stats()
# {
#   'obstacles_avoided': 38,
#   'obstacle_avoidance_enabled': True,
#   'obstacle_stats': {...}
# }
```

### Safety Controller Stats
```python
stats = safe_motor.get_stats()
# {
#   'commands_sent': 1523,
#   'commands_modified': 142,
#   'commands_blocked': 12,
#   'modification_rate': 9.3,
#   'blocking_rate': 0.8
# }
```

## Troubleshooting

### Vấn đề 1: Robot quá nhạy cảm, dừng liên tục
**Giải pháp:**
- Tăng `min_safe_distance` (0.3 → 0.4m)
- Giảm `critical_distance` (0.15 → 0.10m)
- Check stereo calibration quality

### Vấn đề 2: Robot không phát hiện vật cản
**Giải pháp:**
- Check disparity map quality (có depth data không?)
- Verify camera calibration (baseline, focal length)
- Check `max_detection_range` setting
- Enable debug logging: `logger.setLevel(logging.DEBUG)`

### Vấn đề 3: FPS thấp
**Giải pháp:**
- Giảm camera resolution (640x480 → 320x240)
- Tăng `time.sleep()` trong main loop
- Optimize stereo matching parameters

### Vấn đề 4: False positives (phát hiện nhầm)
**Giải pháp:**
- Improve stereo calibration
- Adjust morphological operations (kernel size)
- Tăng minimum obstacle size threshold
- Use temporal filtering (obstacle_history)

## API Reference

### ObstacleDetector

#### detect_from_disparity()
```python
result = detector.detect_from_disparity(
    disparity: np.ndarray,        # Disparity map
    camera_matrix: np.ndarray,    # Camera K matrix (3x3)
    baseline: float,              # Stereo baseline (m)
    robot_velocity: Tuple[float, float] = (0.0, 0.0)
) -> Dict
```

**Returns:**
```python
{
    'obstacles': [
        {
            'distance': 0.25,
            'angle': -15.3,
            'size': 342,
            'critical': False
        },
        ...
    ],
    'zone_status': {
        'front_center': {
            'has_obstacle': True,
            'critical': False,
            'min_distance': 0.25,
            'obstacle_count': 1,
            'safe': False
        },
        ...
    },
    'action': {
        'stop_required': False,
        'warning': True,
        'recommended_linear': 0.06,
        'recommended_angular': -0.5,
        'message': 'Obstacle ahead - turning right',
        'severity': 'warning'
    },
    'closest_obstacle_distance': 0.25,
    'clear_directions': ['front_left', 'left']
}
```

#### get_safety_velocity()
```python
safe_linear, safe_angular = detector.get_safety_velocity(
    desired_linear: float,
    desired_angular: float,
    detection_result: Dict
) -> Tuple[float, float]
```

#### visualize_obstacles()
```python
vis_img = detector.visualize_obstacles(
    image: np.ndarray,
    detection_result: Dict
) -> np.ndarray
```

## Examples

### Example 1: Basic Detection
```python
# Initialize
detector = ObstacleDetector(min_safe_distance=0.3)

# Get disparity from stereo camera
disparity = stereo_camera.get_latest_frames()[2]

# Detect obstacles
result = detector.detect_from_disparity(
    disparity,
    camera_matrix=stereo_camera.K_left,
    baseline=0.06,
    robot_velocity=(0.2, 0.0)
)

# Check result
if result['action']['stop_required']:
    motor.stop()
    print("EMERGENCY STOP!")
elif result['action']['warning']:
    safe_v, safe_w = result['action']['recommended_linear'], result['action']['recommended_angular']
    motor.send_velocity(safe_v, safe_w)
    print(f"Adjusting velocity: {result['action']['message']}")
```

### Example 2: With Autonomous Navigation
```python
# Setup mapper with obstacle avoidance
mapper = AutonomousMapper(
    motor_controller=motor,
    slam_system=slam,
    occupancy_grid=grid,
    obstacle_detector=ObstacleDetector()
)

# Start exploration (automatic obstacle avoidance)
mapper.start_exploration()

# Monitor
while mapper.running:
    stats = mapper.get_stats()
    print(f"Obstacles avoided: {stats['obstacles_avoided']}")
    time.sleep(1.0)
```

### Example 3: Manual Control with Safety
```python
from safety_controller import SafetyVelocityController

# Wrap motor
safe_motor = SafetyVelocityController(motor, detector)

# Continuous update loop
while running:
    # Update detection
    result = detector.detect_from_disparity(disparity, K, baseline, current_vel)
    safe_motor.update_obstacle_detection(result)
    
    # User command (from joystick/keyboard)
    user_linear, user_angular = get_user_input()
    
    # Send (automatically safe)
    safe_motor.send_velocity(user_linear, user_angular)
```

## Future Enhancements

**Planned features:**
- [ ] Dynamic speed adaptation (slow down gradually vs hard stop)
- [ ] Predictive obstacle avoidance (trajectory planning)
- [ ] Multi-level obstacle classification (walls vs movable objects)
- [ ] Integration with occupancy grid for global planning
- [ ] Learning-based obstacle avoidance policies
- [ ] Support for additional sensors (ultrasonic, IR, LiDAR)

---

**Version:** 1.0  
**Last Updated:** 2026-02-19  
**Author:** Robot SLAM Ver2 Team
