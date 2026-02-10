# Robot Control System - Jetson Nano + RP2040

Hệ thống điều khiển robot sử dụng Jetson Nano (high-level processing) và RP2040 (low-level motor control).

## Tính năng

- ✅ Điều khiển động cơ Stepper qua RP2040 với PIO
- ✅ Xử lý ảnh Stereo Camera (OV9732)
- ✅ Visual Odometry với ORB-SLAM3
- ✅ Web Interface để điều khiển và giám sát
- ✅ Giao tiếp Serial JSON-based
- ✅ Safety features (watchdog, velocity limiting)

## Kiến trúc hệ thống

```
┌─────────────────────────────────────┐
│         Jetson Nano (Python)        │
│  ┌──────────┐  ┌──────────────────┐ │
│  │  Camera  │  │   ORB-SLAM3      │ │
│  │ OV9732×2 │→ │ Visual Odometry  │ │
│  └──────────┘  └──────────────────┘ │
│         ↓                            │
│  ┌─────────────────────────────┐    │
│  │    Web Server (Flask)       │    │
│  │  Control UI + Video Stream  │    │
│  └─────────────────────────────┘    │
│         ↓                            │
│  ┌─────────────────────────────┐    │
│  │  Serial Controller (JSON)   │    │
│  └─────────────────────────────┘    │
└──────────────┬──────────────────────┘
               │ USB Serial
┌──────────────┴──────────────────────┐
│         RP2040 (MicroPython)        │
│  ┌─────────────────────────────┐    │
│  │   PIO Stepper Controller    │    │
│  │  Left Motor  │  Right Motor │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

## Yêu cầu phần cứng

- Jetson Nano
- RP2040 (Raspberry Pi Pico)
- 2× Camera OV9732 (Stereo)
- 2× NEMA17 Stepper Motors
- 2× A4988 Motor Drivers

## Cài đặt

### 1. Cài đặt dependencies

```bash
# Cập nhật system
sudo apt update
sudo apt upgrade

# Cài đặt Python dependencies
pip3 install -r requirements.txt

# Cài đặt OpenCV với CUDA support (optional, for better performance)
# Follow: https://github.com/JetsonHacksNano/buildOpenCV
```

### 2. Chuẩn bị ORB-SLAM3

```bash
# Nếu chưa build ORB-SLAM3, thực hiện:
cd /home/jetson/ORB_SLAM3
./build.sh

# Build stereo example
cd Examples/Stereo
mkdir build
cd build
cmake ..
make -j4
```

### 3. Camera Calibration

Nếu chưa có file `calibration.npz`, cần calibrate stereo camera:

```bash
# Sử dụng script calibration (tạo riêng hoặc dùng OpenCV)
python3 calibrate_stereo.py
```

### 4. Upload code lên RP2040

```bash
# Sử dụng Thonny hoặc mpremote để upload motor_controller_V2.py
# Copy file vào RP2040 với tên main.py để tự động chạy khi khởi động
```

### 5. Kiểm tra kết nối

```bash
# Kiểm tra RP2040 đã kết nối
ls /dev/ttyACM*

# Kiểm tra cameras
v4l2-ctl --list-devices

# Test từng component
python3 test_system.py
```

## Sử dụng

### Chạy hệ thống

```bash
# Chạy main program
python3 main.py
```

### Truy cập Web Interface

Mở browser và truy cập:
```
http://<jetson-ip>:5000
```

Tính năng:
- 📹 Live video stream từ stereo cameras
- 🎮 Virtual joystick để điều khiển
- 📊 Real-time status monitoring
- ⌨️ Keyboard control (W/A/S/D hoặc arrow keys)

### Điều khiển qua API

```python
import requests

# Enable motors
requests.post('http://localhost:5000/api/enable')

# Set velocity (linear: m/s, angular: rad/s)
requests.post('http://localhost:5000/api/velocity', 
              json={'linear': 0.2, 'angular': 0.0})

# Emergency stop
requests.post('http://localhost:5000/api/stop')

# Disable motors
requests.post('http://localhost:5000/api/disable')
```

### Keyboard Controls

- `W` / `↑` : Di chuyển tiến
- `S` / `↓` : Di chuyển lùi
- `A` / `←` : Rẽ trái
- `D` / `→` : Rẽ phải
- `Space` : Emergency stop

## Cấu trúc thư mục

```
.
├── main.py                    # Main program
├── serial_controller.py       # RP2040 communication
├── stereo_camera.py          # Camera handling
├── orbslam_interface.py      # ORB-SLAM3 wrapper
├── web_server.py             # Flask web server
├── test_system.py            # System tests
├── requirements.txt          # Python dependencies
├── config.yaml               # Configuration (optional)
├── calibration.npz           # Camera calibration
├── templates/
│   └── index.html           # Web UI
└── motor_controller_V2.py   # RP2040 MicroPython code
```

## Configuration

Chỉnh sửa trong `main.py`:

```python
config = {
    'serial_port': '/dev/ttyACM0',      # RP2040 port
    'camera_left_id': 0,                # Left camera device
    'camera_right_id': 1,               # Right camera device
    'camera_width': 1280,
    'camera_height': 720,
    'camera_fps': 30,
    'max_linear_speed': 0.5,            # m/s
    'max_angular_speed': 2.0,           # rad/s
    'orbslam_path': '/home/jetson/ORB_SLAM3',
    'use_orbslam': True,                # False to use simple VO
    'webserver_port': 5000,
}
```

## Troubleshooting

### RP2040 không kết nối

```bash
# Kiểm tra device
ls -l /dev/ttyACM*

# Thêm user vào dialout group
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Camera không hoạt động

```bash
# Kiểm tra cameras
v4l2-ctl --list-devices

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).read())"
```

### ORB-SLAM3 lỗi

```bash
# Kiểm tra vocabulary
ls -lh /home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt

# Kiểm tra settings file
cat orbslam_stereo.yaml
```

### Hiệu suất thấp

```bash
# Enable Jetson performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Giảm resolution camera
# Tắt ORB-SLAM3 viewer
# Giảm FPS web stream
```

## Safety Features

1. **Watchdog Timer**: Tự động dừng nếu không nhận lệnh trong 500ms
2. **Velocity Limiting**: Giới hạn tốc độ tối đa
3. **Emergency Stop**: Dừng ngay lập tức qua button hoặc Space key
4. **Enable/Disable**: Motors phải được enable trước khi di chuyển

## License

MIT License

## Tác giả

Robot Control System for Jetson Nano + RP2040

## Tham khảo

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [OpenCV Stereo Calibration](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
- [Flask Documentation](https://flask.palletsprojects.com/)
- [RP2040 PIO](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
