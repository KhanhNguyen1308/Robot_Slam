# QUICK START GUIDE

## Bước 1: Cài đặt Dependencies

```bash
# Trên Jetson Nano
pip3 install -r requirements.txt
```

## Bước 2: Upload Code lên RP2040

1. Kết nối RP2040 qua USB
2. Sử dụng Thonny IDE hoặc mpremote:

```bash
# Sử dụng mpremote (nếu đã cài)
pip3 install mpremote
mpremote connect /dev/ttyACM0 cp motor_controller_V2.py :main.py
```

Hoặc dùng Thonny:
- Mở `motor_controller_V2.py` (file đã upload)
- File > Save As > Raspberry Pi Pico > main.py

## Bước 3: Calibrate Stereo Camera

```bash
# Bước 1: Capture ảnh calibration (20 ảnh)
python3 calibrate_stereo.py --capture --num-images 20

# In checkerboard pattern (9x6) và di chuyển trước camera
# Nhấn SPACE để chụp mỗi ảnh, ESC để thoát

# Bước 2: Chạy calibration
python3 calibrate_stereo.py --calibrate

# Bước 3: Kiểm tra kết quả
python3 calibrate_stereo.py --test
```

## Bước 4: Test Từng Component

```bash
python3 test_system.py
```

Kiểm tra:
- ✓ Dependencies installed
- ✓ Serial (RP2040) connected
- ✓ Cameras working
- ✓ ORB-SLAM3 available

## Bước 5: Chạy Hệ Thống

```bash
python3 main.py
```

Output:
```
INFO - Connected to RP2040 on /dev/ttyACM0
INFO - Cameras opened: 1280x720@30fps
INFO - ORB-SLAM3 started
INFO - Web server starting on http://0.0.0.0:5000
============================================================
Robot system running
Web interface: http://localhost:5000
Press Ctrl+C to stop
============================================================
```

## Bước 6: Truy Cập Web Interface

Mở browser:
```
http://<jetson-ip>:5000
```

hoặc từ chính Jetson Nano:
```
http://localhost:5000
```

## Bước 7: Điều Khiển Robot

### Qua Web Interface:
1. Nhấn "Enable Motors"
2. Sử dụng virtual joystick để điều khiển
3. Hoặc dùng keyboard: W/A/S/D

### Qua Python API:
```python
python3 examples.py
```

Chọn ví dụ:
1. Basic Movement
2. Rotation
3. Square Path
4. Circle Path
5. Smooth Acceleration
6. Figure-8
7. Status Monitoring
8. Obstacle Avoidance Pattern

### Qua Code:
```python
from examples import RobotClient

robot = RobotClient()
robot.enable()
robot.set_velocity(0.2, 0)  # Linear: 0.2 m/s, Angular: 0 rad/s
time.sleep(2)
robot.set_velocity(0, 0)
robot.disable()
```

## Troubleshooting

### Lỗi: Cannot connect to RP2040
```bash
# Kiểm tra device
ls -l /dev/ttyACM*

# Thêm quyền
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Lỗi: Camera not found
```bash
# Kiểm tra cameras
v4l2-ctl --list-devices
ls -l /dev/video*

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).read()[0])"
```

### Lỗi: ORB-SLAM3 not found
```bash
# Kiểm tra path
ls -la /home/jetson/ORB_SLAM3

# Build nếu chưa có
cd /home/jetson/ORB_SLAM3
./build.sh
```

### Robot chạy chậm
```bash
# Enable performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Hoặc giảm resolution camera trong config
# Hoặc tắt ORB-SLAM3 viewer
```

## Configuration

Chỉnh sửa trong `main.py`:

```python
config = {
    'serial_port': '/dev/ttyACM0',      # Thay đổi nếu khác
    'camera_left_id': 0,                # ID camera trái
    'camera_right_id': 1,               # ID camera phải
    'camera_width': 1280,               # Độ phân giải
    'camera_height': 720,
    'max_linear_speed': 0.5,            # m/s (giới hạn tốc độ)
    'max_angular_speed': 2.0,           # rad/s
    'use_orbslam': True,                # False = simple VO
    'orbslam_viewer': False,            # True nếu có display
}
```

## Safety Notes

⚠️ **LƯU Ý AN TOÀN:**
- Luôn test trong không gian rộng rãi
- Sẵn sàng nhấn EMERGENCY STOP
- Watchdog sẽ tự động dừng sau 500ms nếu mất kết nối
- Giới hạn tốc độ được enforce bởi SafetyController

## Files Structure

```
.
├── main.py                   # Main program - CHẠY FILE NÀY
├── serial_controller.py      # RP2040 communication
├── stereo_camera.py         # Camera handling
├── orbslam_interface.py     # ORB-SLAM3 wrapper
├── web_server.py            # Flask web server
├── examples.py              # Example scripts
├── test_system.py           # System tests
├── calibrate_stereo.py      # Camera calibration
├── requirements.txt         # Dependencies
├── README.md                # Full documentation
├── QUICKSTART.md            # This file
└── templates/
    └── index.html          # Web UI template
```

## Next Steps

1. ✅ Calibrate camera tốt hơn với nhiều ảnh hơn (30-40)
2. ✅ Tune ORB-SLAM3 parameters trong `orbslam_stereo.yaml`
3. ✅ Thêm autonomous navigation
4. ✅ Tích hợp sensors khác (IMU, Lidar, etc.)
5. ✅ Implement path planning algorithms

## Support

Nếu gặp vấn đề:
1. Chạy `python3 test_system.py` để kiểm tra
2. Kiểm tra logs trong terminal
3. Xem README.md để biết thêm chi tiết
