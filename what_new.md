# 🎉 CẬP NHẬT HỆ THỐNG - VERSION 2.0

## ✨ TÍNH NĂNG MỚI

### 1. **IMU Integration (ADXL345)** 🎯
- **Hardware**: ADXL345 3-axis accelerometer qua I2C bus 1
- **File**: `imu_module.py`
- **Chức năng**:
  - ✅ Đo góc nghiêng (Pitch & Roll)
  - ✅ Phát hiện va chạm (collision detection)
  - ✅ Cảnh báo nghiêng quá mức (>20°)
  - ✅ Complementary filter để lọc nhiễu
  - ✅ Velocity estimation (dead reckoning)

```python
# Sử dụng IMU
from imu_module import ADXL345, IMUFilter

imu = ADXL345(bus=1)
imu.connect()
imu.calibrate()

filter = IMUFilter(imu)
filter.start()

pitch, roll = filter.get_orientation()
collision = filter.detect_collision(threshold=2.5)
tilted = filter.is_tilted(max_angle=20.0)
```

### 2. **Obstacle Detection** 🚧
- **File**: `obstacle_detection.py`
- **Phương pháp**: Stereo depth mapping
- **Chức năng**:
  - ✅ Tính toán depth map từ stereo camera
  - ✅ Phát hiện vật cản theo khoảng cách
  - ✅ Phân loại mức độ nguy hiểm (high/medium/low)
  - ✅ Phân tích 3 vùng (left/center/right)
  - ✅ Gợi ý hướng di chuyển an toàn

```python
# Sử dụng Obstacle Detection
from obstacle_detection import ObstacleDetector

detector = ObstacleDetector(
    camera_baseline=0.06,      # 6cm
    camera_focal_length=700,   # pixels
    danger_zone_distance=0.5   # 50cm
)

# Detect obstacles
disparity = detector.compute_disparity(img_left, img_right)
depth_map = detector.disparity_to_depth(disparity)
obstacles = detector.detect_obstacles(depth_map)

# Get danger zones
zones = detector.get_danger_zones(obstacles)
# zones = {'left': 'clear', 'center': 'danger', 'right': 'warning'}

# Get safe direction
direction, confidence = detector.get_safe_direction(obstacles)
# direction = 'left', confidence = 0.8
```

### 3. **Autonomous Navigation** 🤖
- **File**: Integrated trong `main.py`
- **Chức năng**:
  - ✅ Tự động tránh vật cản
  - ✅ Reactive behavior (phản ứng tức thì)
  - ✅ Chế độ manual/auto switching
  - ✅ Emergency auto-stop

```python
# Enable autonomous mode via API
POST /api/autonomous
{
  "enable": true
}

# Robot tự động di chuyển và tránh vật cản
```

### 4. **Camera Sync Fix** 📷
- **Vấn đề**: Camera bị flicker với đèn 50Hz
- **Giải pháp**: Tự động set power_line_frequency=1
- **File**: `stereo_camera.py`

```python
# Tự động thực hiện khi open camera:
os.system(f"v4l2-ctl -d /dev/video{cam_index} --set-ctrl=power_line_frequency=1")
```

## 📦 FILES MỚI

1. **imu_module.py** (347 dòng)
   - Class ADXL345 - Driver cho accelerometer
   - Class IMUFilter - Filtering và sensor fusion
   - Collision detection
   - Tilt monitoring

2. **obstacle_detection.py** (428 dòng)
   - Class ObstacleDetector - Stereo depth + detection
   - Class SimpleObstacleAvoidance - Reactive avoidance
   - Obstacle classification
   - Danger zone analysis

3. **demo_autonomous.py** (362 dòng)
   - 4 demos cho autonomous navigation
   - Obstacle monitoring
   - IMU monitoring
   - Full autonomous mode

## 🔄 FILES CẬP NHẬT

1. **main.py**
   - Thêm IMU integration
   - Thêm obstacle detection
   - Autonomous mode support
   - Collision handling
   - Enhanced logging

2. **stereo_camera.py**
   - Power line frequency fix
   - Camera sync improvement

3. **web_server.py**
   - API endpoint `/api/imu`
   - API endpoint `/api/obstacles`
   - API endpoint `/api/autonomous`
   - Video stream `/video/obstacles`
   - Video stream `/video/depth`

4. **requirements.txt**
   - Thêm `smbus2` cho I2C communication

## 🎮 CÁC API MỚI

### GET /api/imu
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

### GET /api/obstacles
```json
{
  "obstacles": [
    {"distance": 0.45, "angle": -5.2, "severity": "high"}
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

### POST /api/autonomous
```json
{
  "enable": true
}
```

## 🎬 VIDEO STREAMS MỚI

- `/video/obstacles` - Left camera với obstacle overlay
- `/video/depth` - Depth map visualization

## 🚀 SỬ DỤNG

### Test IMU
```bash
python3 imu_module.py
```

### Test Obstacle Detection
```bash
python3 obstacle_detection.py
```

### Run Full System
```bash
python3 main.py
```

### Run Autonomous Demos
```bash
python3 demo_autonomous.py
```

### Enable I2C (nếu chưa có)
```bash
sudo apt-get install i2c-tools python3-smbus
sudo raspi-config
# Interface Options → I2C → Enable

# Test I2C
i2cdetect -y 1
# Should see 0x53
```

## 📊 ROBOT HIỆN TẠI CÓ THỂ

### ✅ Sensing
- Stereo vision (1280x720 @ 30fps)
- Visual SLAM (ORB-SLAM3)
- **IMU orientation (50Hz)**
- **Collision detection**
- **Depth sensing**
- **Obstacle detection**

### ✅ Control
- Manual joystick/keyboard
- Web interface
- Python API
- REST API
- **Autonomous mode**

### ✅ Safety
- Watchdog timer
- Velocity limiting
- Emergency stop
- **Collision auto-stop**
- **Tilt warning**
- **Obstacle auto-stop**

### ✅ Autonomous
- **Reactive obstacle avoidance**
- **Safe direction finding**
- **Auto emergency brake**
- ⏳ Path planning (future)
- ⏳ Goal navigation (future)

## 🎯 DEMO SCENARIOS

### Demo 1: Obstacle Monitoring
Robot di chuyển tiến, liên tục scan obstacles, dừng nếu có vật cản trước mặt.

### Demo 2: Full Autonomous
Enable autonomous mode, robot tự navigate, tránh vật cản, tìm đường tự động.

### Demo 3: IMU Monitoring
Di chuyển theo pattern, monitor pitch/roll/collision real-time.

### Demo 4: Reactive Avoidance
Thực hiện pattern tránh vật cản: forward → detect → turn → forward.

## 🔧 CONFIGURATION

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

## 📈 PERFORMANCE

- Camera: 30 FPS
- SLAM: 15-20 FPS
- **Obstacle Detection: 10-15 FPS**
- **IMU: 50 Hz**
- Control loop: 50 Hz

## 🐛 TROUBLESHOOTING

### IMU không hoạt động
```bash
# Check I2C
i2cdetect -y 1

# Check permissions
sudo usermod -a -G i2c $USER
```

### Obstacle detection không chính xác
```bash
# Recalibrate camera
python3 calibrate_stereo.py --capture
python3 calibrate_stereo.py --calibrate
```

### Camera flicker
- Đã fix tự động trong stereo_camera.py
- Power line frequency = 1 (50Hz)

## 📚 DOCUMENTATION

- **README_V2.md** - Full documentation
- **QUICKSTART.md** - Quick start guide
- **OVERVIEW.md** - System overview
- **Inline comments** - Code documentation

## 🎓 LEARNING RESOURCES

### IMU
- ADXL345 datasheet
- Complementary filter
- Dead reckoning

### Obstacle Detection
- Stereo vision basics
- Depth from disparity
- Obstacle classification

### Autonomous Navigation
- Reactive behaviors
- Potential fields
- Path planning (future)

---

**Tổng kết**: Hệ thống giờ đây có khả năng cảm nhận môi trường tốt hơn (IMU + obstacles), tự động tránh vật cản, và phát hiện va chạm. Đây là nền tảng cho autonomous navigation hoàn toàn trong tương lai!

**Next Steps**:
1. ✅ Test IMU trên hardware thực
2. ✅ Calibrate stereo camera chính xác
3. ✅ Tune obstacle detection parameters
4. ⏳ Implement path planning (A*, RRT)
5. ⏳ Add more sensors (Lidar, ultrasonic)
6. ⏳ Machine learning navigation