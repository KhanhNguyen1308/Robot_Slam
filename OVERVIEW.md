# HỆ THỐNG ĐIỀU KHIỂN ROBOT - JETSON NANO + RP2040

## 📋 TỔNG QUAN

Hệ thống điều khiển robot hoàn chỉnh với:
- **High-Level Processing**: Jetson Nano (Python)
- **Low-Level Motor Control**: RP2040 (MicroPython)
- **Vision**: Stereo Camera OV9732 + ORB-SLAM3
- **Interface**: Web-based control panel

## 📦 DANH SÁCH FILES

### Core System Files
1. **main.py** - Main program, khởi động toàn bộ hệ thống
2. **serial_controller.py** - Giao tiếp Serial với RP2040
3. **stereo_camera.py** - Xử lý Stereo Camera
4. **orbslam_interface.py** - Tích hợp ORB-SLAM3
5. **web_server.py** - Flask web server

### Utility Scripts
6. **test_system.py** - Test từng component
7. **calibrate_stereo.py** - Calibration tool cho stereo camera
8. **examples.py** - Ví dụ điều khiển robot

### Configuration & Documentation
9. **config.yaml** - File cấu hình mẫu
10. **requirements.txt** - Python dependencies
11. **README.md** - Documentation đầy đủ
12. **QUICKSTART.md** - Hướng dẫn nhanh

### Web Interface
13. **templates/index.html** - Web UI với virtual joystick

### RP2040 Code
14. **motor_controller_V2.py** - Code đã có sẵn (upload lên RP2040)

## 🚀 HƯỚNG DẪN SỬ DỤNG NHANH

### 1. Cài đặt
```bash
pip3 install -r requirements.txt
```

### 2. Upload code lên RP2040
```bash
# Dùng Thonny hoặc mpremote
mpremote connect /dev/ttyACM0 cp motor_controller_V2.py :main.py
```

### 3. Calibrate Camera
```bash
python3 calibrate_stereo.py --capture    # Chụp ảnh
python3 calibrate_stereo.py --calibrate  # Chạy calibration
python3 calibrate_stereo.py --test       # Test kết quả
```

### 4. Test Hệ Thống
```bash
python3 test_system.py
```

### 5. Chạy Robot
```bash
python3 main.py
```

### 6. Truy cập Web Interface
```
http://<jetson-ip>:5000
```

## 🏗️ KIẾN TRÚC HỆ THỐNG

```
┌─────────────────────────────────────────────────────────┐
│                    JETSON NANO                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Camera     │  │  ORB-SLAM3   │  │ Web Server   │  │
│  │  OV9732 x2   │→ │    SLAM      │  │   Flask      │  │
│  │  1280x720    │  │ Trajectory   │  │  Port 5000   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│         ↓                 ↓                  ↑           │
│  ┌─────────────────────────────────────────────────┐    │
│  │         Main Controller (main.py)               │    │
│  │  - Frame processing loop                        │    │
│  │  - SLAM integration                             │    │
│  │  - Command distribution                         │    │
│  └─────────────────────────────────────────────────┘    │
│         ↓                                                │
│  ┌─────────────────────────────────────────────────┐    │
│  │   Serial Controller (JSON Protocol)             │    │
│  │  - Velocity commands                            │    │
│  │  - Safety watchdog                              │    │
│  │  - Status monitoring                            │    │
│  └─────────────────────────────────────────────────┘    │
└────────────────────────┬────────────────────────────────┘
                         │ USB Serial (115200 baud)
                         ↓
┌─────────────────────────────────────────────────────────┐
│                     RP2040 (PICO)                       │
│  ┌─────────────────────────────────────────────────┐    │
│  │       PIO-based Stepper Controller              │    │
│  │  - State Machine 0: Left Motor                  │    │
│  │  - State Machine 1: Right Motor                 │    │
│  │  - Differential drive kinematics                │    │
│  │  - Watchdog safety (500ms timeout)              │    │
│  └─────────────────────────────────────────────────┘    │
│         ↓                           ↓                    │
│  ┌─────────────┐            ┌─────────────┐             │
│  │ Left Motor  │            │ Right Motor │             │
│  │  NEMA17     │            │  NEMA17     │             │
│  │  + A4988    │            │  + A4988    │             │
│  └─────────────┘            └─────────────┘             │
└─────────────────────────────────────────────────────────┘
```

## 🎮 TÍNH NĂNG

### Vision System
- ✅ Stereo camera capture (OV9732)
- ✅ Camera calibration tool
- ✅ Stereo rectification
- ✅ ORB-SLAM3 integration
- ✅ Visual odometry fallback
- ✅ Trajectory logging

### Motor Control
- ✅ PIO-based stepper control (high precision)
- ✅ Differential drive kinematics
- ✅ JSON-based command protocol
- ✅ Velocity limiting
- ✅ Watchdog safety
- ✅ Emergency stop

### Web Interface
- ✅ Live video streaming
- ✅ Virtual joystick control
- ✅ Keyboard control (WASD)
- ✅ Real-time status monitoring
- ✅ Responsive design
- ✅ REST API

### Python API
- ✅ Simple client library
- ✅ 8 example control patterns
- ✅ Status monitoring
- ✅ Trajectory export

## 📊 THÔNG SỐ KỸ THUẬT

### Robot Mechanics
- Track width: 275mm
- Sprocket: 20 teeth, 15mm pitch
- Gear ratio: 20:68
- Steps per meter: ~10,588 (with 1/16 microstepping)

### Motion Limits
- Max linear speed: 0.5 m/s (configurable)
- Max angular speed: 2.0 rad/s (configurable)
- Watchdog timeout: 500ms

### Camera
- Resolution: 1280x720 (configurable)
- Frame rate: 30 FPS
- Format: MJPEG
- Baseline: ~auto-detected from calibration

### Communication
- Protocol: JSON over Serial
- Baud rate: 115200
- Commands: velocity, enable, disable, stop, status

## 🔧 CẤU HÌNH

Các thông số có thể điều chỉnh trong `main.py`:

```python
config = {
    # Serial
    'serial_port': '/dev/ttyACM0',
    'serial_baudrate': 115200,
    
    # Camera
    'camera_left_id': 0,
    'camera_right_id': 1,
    'camera_width': 1280,
    'camera_height': 720,
    'camera_fps': 30,
    'calibration_file': 'calibration.npz',
    
    # SLAM
    'use_orbslam': True,
    'orbslam_path': '/home/jetson/ORB_SLAM3',
    'orbslam_viewer': False,
    
    # Limits
    'max_linear_speed': 0.5,
    'max_angular_speed': 2.0,
    
    # Web
    'webserver_port': 5000,
}
```

## 🛡️ SAFETY FEATURES

1. **Watchdog Timer**: Tự động dừng nếu mất kết nối > 500ms
2. **Velocity Limiting**: Hardware limits trong SafetyController
3. **Emergency Stop**: Button và Space key
4. **Enable/Disable**: Motors must be explicitly enabled
5. **Connection Monitoring**: Auto-reconnect on serial errors

## 📈 PERFORMANCE

### Jetson Nano (Default Settings)
- Camera processing: ~30 FPS
- SLAM tracking: ~15-20 FPS
- Web streaming: 15 FPS
- Control loop: 50 Hz

### Optimization Tips
```bash
# Enable max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Reduce resolution if needed
camera_width: 640
camera_height: 480

# Disable SLAM viewer
orbslam_viewer: False
```

## 🐛 TROUBLESHOOTING

### Common Issues

**Issue**: Cannot connect to RP2040
```bash
ls -l /dev/ttyACM*
sudo usermod -a -G dialout $USER
```

**Issue**: Camera not working
```bash
v4l2-ctl --list-devices
python3 -c "import cv2; print(cv2.VideoCapture(0).read()[0])"
```

**Issue**: ORB-SLAM3 errors
```bash
ls /home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt
cat orbslam_stereo.yaml
```

**Issue**: Slow performance
- Enable Jetson performance mode
- Reduce camera resolution
- Disable SLAM viewer
- Lower web stream FPS

## 📚 EXAMPLES

### Example 1: Basic Movement
```python
from examples import RobotClient
import time

robot = RobotClient()
robot.enable()
robot.set_velocity(0.2, 0)  # Forward
time.sleep(2)
robot.set_velocity(0, 0)    # Stop
robot.disable()
```

### Example 2: Square Path
```python
python3 examples.py
# Chọn option 3: Square Path
```

### Example 3: Web Control
```javascript
// In browser console at http://jetson:5000
fetch('/api/velocity', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({linear: 0.2, angular: 0})
});
```

## 🔮 FUTURE ENHANCEMENTS

- [ ] Path planning algorithms
- [ ] Obstacle detection & avoidance
- [ ] IMU integration
- [ ] Lidar support
- [ ] Multi-robot coordination
- [ ] ROS integration
- [ ] Machine learning for navigation

## 📝 LICENSE

MIT License - Feel free to use and modify

## 👥 SUPPORT

Xem README.md để biết thêm chi tiết
Chạy test_system.py để debug
Kiểm tra logs trong terminal

---

**Tác giả**: Robot Control System
**Phiên bản**: 1.0
**Ngày**: February 2026
