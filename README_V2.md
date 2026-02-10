# HỆ THỐNG ĐIỀU KHIỂN ROBOT - V2.0 (IMU + OBSTACLE DETECTION)

## 🆕 CẬP NHẬT MỚI

### Version 2.0 Features:
- ✅ **IMU Integration** - ADXL345 accelerometer (I2C bus 1)
- ✅ **Obstacle Detection** - Stereo depth-based detection
- ✅ **Autonomous Navigation** - Automatic obstacle avoidance
- ✅ **Collision Detection** - Real-time collision sensing via IMU
- ✅ **Tilt Monitoring** - Safety shutdown on excessive tilt
- ✅ **Camera Sync Fix** - Power line frequency adjustment (50Hz Vietnam)

## 📋 HARDWARE REQUIREMENTS

### Core Components
- Jetson Nano
- RP2040 (Raspberry Pi Pico)
- 2× Camera OV9732 (Stereo)
- 2× NEMA17 Stepper Motors
- 2× A4988 Motor Drivers

### New Sensors
- **ADXL345** 3-axis accelerometer (I2C address 0x53, Bus 1)

## 🚀 QUICK START

### 1. Cài đặt Dependencies
```bash
pip3 install -r requirements.txt

# Install I2C tools
sudo apt-get install i2c-tools python3-smbus

# Enable I2C
sudo raspi-config
# Interface Options → I2C → Enable
```

### 2. Test IMU
```bash
# Kiểm tra I2C device
i2cdetect -y 1
# Should see 0x53

# Test IMU
python3 imu_module.py
```

### 3. Calibrate Camera
```bash
python3 calibrate_stereo.py --capture
python3 calibrate_stereo.py --calibrate
python3 calibrate_stereo.py --test
```

### 4. Run System
```bash
python3 main.py
```

### 5. Web Interface
```
http://<jetson-ip>:5000
```

## 🎮 CÁC CHỨC NĂNG MỚI

### 1. IMU Features
- **Orientation Tracking** - Pitch and roll angles
- **Collision Detection** - Detects sudden impacts
- **Tilt Monitoring** - Warns if robot tips over
- **Acceleration Monitoring** - Track movement dynamics

### 2. Obstacle Detection
- **Stereo Depth Mapping** - Real-time depth estimation
- **Obstacle Classification** - Distance and severity levels
- **Danger Zones** - Left/Center/Right zone analysis
- **Safe Direction** - Recommends best movement direction

### 3. Autonomous Navigation
- **Auto Obstacle Avoidance** - Reactive avoidance behavior
- **Manual/Auto Mode** - Switch between control modes
- **Safety Features** - Auto-stop on close obstacles

## 📊 API ENDPOINTS (NEW)

### IMU Data
```bash
GET /api/imu
```
Response:
```json
{
  "connected": true,
  "pitch": 2.5,
  "roll": -1.2,
  "accel_x": 0.02,
  "accel_y": -0.01,
  "accel_z": 0.98,
  "magnitude": 0.99,
  "collision": false
}
```

### Obstacles
```bash
GET /api/obstacles
```
Response:
```json
{
  "obstacles": [
    {
      "distance": 0.45,
      "angle": -5.2,
      "severity": "high"
    }
  ],
  "danger_zones": {
    "left": "clear",
    "center": "danger",
    "right": "warning"
  },
  "safe_direction": "left",
  "confidence": 0.8
}
```

### Autonomous Mode
```bash
POST /api/autonomous
Content-Type: application/json

{
  "enable": true
}
```

## 🎬 VIDEO STREAMS (NEW)

- `/video/left` - Left camera
- `/video/right` - Right camera
- `/video/stereo` - Side-by-side
- `/video/obstacles` - **NEW** With obstacle overlay
- `/video/depth` - **NEW** Depth map visualization

## 🤖 AUTONOMOUS DEMOS

### Run Demos
```bash
chmod +x demo_autonomous.py
python3 demo_autonomous.py
```

Available demos:
1. **Obstacle Monitoring** - Monitor obstacles while moving
2. **Autonomous Navigation** - Full auto mode (30s)
3. **IMU Monitoring** - Track tilt during movement
4. **Obstacle Avoidance Pattern** - Reactive avoidance

### Example: Enable Autonomous Mode
```python
from demo_autonomous import AutonomousRobot

robot = AutonomousRobot()
robot.enable_motors()
robot.set_autonomous(True)  # Start auto navigation

# Robot will now avoid obstacles automatically
time.sleep(30)

robot.set_autonomous(False)
robot.disable_motors()
```

## ⚙️ CONFIGURATION

### New Config Options (main.py)
```python
config = {
    # IMU
    'use_imu': True,
    'imu_bus': 1,
    'imu_address': 0x53,
    
    # Obstacle Detection
    'danger_zone_distance': 0.5,  # meters
    
    # ... existing config ...
}
```

### Obstacle Detection Parameters
```python
detector = ObstacleDetector(
    camera_baseline=0.06,          # 6cm between cameras
    camera_focal_length=700,       # pixels
    min_distance=0.2,              # 20cm minimum
    max_distance=3.0,              # 3m maximum
    danger_zone_distance=0.5       # 50cm danger threshold
)
```

## 🛡️ SAFETY FEATURES (ENHANCED)

### Original Features
1. Watchdog timer (500ms)
2. Velocity limiting
3. Emergency stop
4. Enable/disable control

### New Features
5. **Collision Detection** - Auto-stop on impact
6. **Tilt Monitoring** - Warn on 20° tilt
7. **Obstacle Auto-Stop** - Stop if obstacle <30cm
8. **Autonomous Emergency** - Auto-brake in danger zone

## 📈 PERFORMANCE

### Typical Performance (Jetson Nano)
- Camera FPS: ~30
- SLAM tracking: ~15-20 FPS
- Obstacle detection: ~10-15 FPS
- IMU update rate: 50 Hz
- Control loop: 50 Hz

### Optimization Tips
```bash
# Max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Lower camera resolution
camera_width: 640
camera_height: 480

# Disable SLAM viewer
orbslam_viewer: False

# Use BM instead of SGBM for stereo
use_sgbm: False  # in obstacle detection
```

## 🔧 TROUBLESHOOTING

### IMU Issues
```bash
# Check I2C
i2cdetect -y 1

# Check permissions
sudo usermod -a -G i2c $USER
# Logout and login

# Test IMU directly
python3 -c "from imu_module import ADXL345; imu = ADXL345(); print(imu.connect())"
```

### Obstacle Detection Issues
```bash
# Check calibration
python3 -c "import numpy as np; calib = np.load('calibration.npz'); print(calib.files)"

# Test obstacle detection
python3 obstacle_detection.py
```

### Camera Flicker (Fixed)
Camera flicker with 50Hz lights is now automatically fixed via:
```bash
v4l2-ctl -d /dev/video0 --set-ctrl=power_line_frequency=1
```
This is applied automatically in `stereo_camera.py`.

## 📝 FILES STRUCTURE (UPDATED)

```
.
├── main.py                    # Main program with IMU + obstacles
├── serial_controller.py       # RP2040 communication
├── stereo_camera.py          # Camera (with 50Hz fix)
├── imu_module.py             # NEW - ADXL345 IMU
├── obstacle_detection.py     # NEW - Obstacle detection
├── orbslam_interface.py      # ORB-SLAM3 wrapper
├── web_server.py             # Flask (updated with new endpoints)
├── demo_autonomous.py        # NEW - Autonomous demos
├── examples.py               # Example scripts
├── test_system.py            # System tests
├── calibrate_stereo.py       # Camera calibration
├── requirements.txt          # Dependencies (updated)
├── README.md                 # This file
├── QUICKSTART.md             # Quick start guide
├── OVERVIEW.md               # System overview
└── templates/
    └── index.html            # Web UI
```

## 🎯 WHAT ROBOT CAN DO NOW

### Manual Control
- ✅ Joystick/keyboard control
- ✅ Web interface
- ✅ Python API
- ✅ REST API

### Sensing
- ✅ Stereo vision
- ✅ SLAM/Visual odometry
- ✅ **IMU orientation**
- ✅ **Collision detection**
- ✅ **Depth sensing**
- ✅ **Obstacle detection**

### Autonomous Capabilities
- ✅ **Reactive obstacle avoidance**
- ✅ **Auto-stop on danger**
- ✅ **Safe direction recommendation**
- ⏳ Path planning (future)
- ⏳ Goal-based navigation (future)

## 🔮 FUTURE ENHANCEMENTS

- [ ] Path planning (A*, RRT)
- [ ] SLAM-based navigation
- [ ] Multiple obstacle tracking
- [ ] Lidar integration
- [ ] Multi-robot coordination
- [ ] Learning-based navigation

## 📚 USAGE EXAMPLES

### Example 1: Monitor Obstacles
```python
from demo_autonomous import AutonomousRobot
import time

robot = AutonomousRobot()
robot.enable_motors()

for i in range(10):
    obstacles = robot.get_obstacles()
    print(f"Obstacles: {len(obstacles['obstacles'])}")
    print(f"Danger zones: {obstacles['danger_zones']}")
    print(f"Safe direction: {obstacles['safe_direction']}")
    time.sleep(1)

robot.disable_motors()
```

### Example 2: IMU Monitoring
```python
robot = AutonomousRobot()

for i in range(10):
    imu = robot.get_imu()
    print(f"Pitch: {imu['pitch']:.1f}° Roll: {imu['roll']:.1f}°")
    print(f"Collision: {imu['collision']}")
    time.sleep(0.5)
```

### Example 3: Full Autonomous
```python
robot = AutonomousRobot()
robot.enable_motors()
robot.set_autonomous(True)

# Robot navigates autonomously
time.sleep(60)

robot.set_autonomous(False)
robot.disable_motors()
```

## 🆘 SUPPORT

### Check System
```bash
python3 test_system.py
```

### Check IMU
```bash
python3 imu_module.py
```

### Check Obstacles
```bash
python3 obstacle_detection.py
```

### View Logs
```bash
# Terminal output shows detailed logs
# Check for errors/warnings
```

## 📄 LICENSE

MIT License

## 👥 CONTRIBUTORS

Robot Control System V2.0
- Base system with SLAM
- IMU integration
- Obstacle detection
- Autonomous navigation

---

**Version**: 2.0  
**Date**: February 2026  
**Status**: Production Ready