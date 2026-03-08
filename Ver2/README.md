# Robot Hỗ Trợ Tự Hành — Ver2

Robot bánh xích tự hành cho môi trường trong nhà, có khả năng lập bản đồ, nhận diện vật thể, và điều khiển bằng giọng nói tiếng Việt. Hệ thống kết hợp **Jetson Nano** (thu thập dữ liệu thực tế, điều khiển an toàn) với **Server x99** (suy luận AI nặng qua WiFi nội bộ).

---

## Mục Lục

- [Tổng Quan Hệ Thống](#tổng-quan-hệ-thống)
- [Cấu Hình Phần Cứng](#cấu-hình-phần-cứng)
- [Kiến Trúc Phần Mềm](#kiến-trúc-phần-mềm)
- [Robot Hiện Tại Có Thể Làm Gì](#robot-hiện-tại-có-thể-làm-gì)
- [Cấu Trúc Dự Án](#cấu-trúc-dự-án)
- [Cấu Hình config.yaml](#cấu-hình-configyaml)
- [Hướng Dẫn Khởi Động](#hướng-dẫn-khởi-động)
- [Giao Diện Web](#giao-diện-web)
- [Hạn Chế Hiện Tại & Kế Hoạch Nâng Cấp](#hạn-chế-hiện-tại--kế-hoạch-nâng-cấp)

---

## Tổng Quan Hệ Thống

Hệ thống chia thành hai lớp xử lý kết nối qua WiFi nội bộ:

```
┌──────────────────────────────────────────────┐   WiFi LAN (192.168.2.x)   ┌──────────────────────────────────────────────────┐
│             JETSON NANO (Robot)              │ ◄────────────────────────► │              SERVER x99                         │
│                                              │                             │                                                  │
│  Lớp an toàn & thực thi (real-time)         │   Ảnh JPEG nén              │  Lớp trí tuệ nhân tạo (suy luận nặng)           │
│  ─────────────────────────────────────       │   ──────────────►          │  ────────────────────────────────────────────    │
│  • Điều khiển động cơ bước (GPIO)            │                             │  • YOLOv11 nhận diện vật thể (MI50 GPU)          │
│  • Dừng khẩn cấp khi có chướng ngại vật     │   JSON detections           │  • [Phase 2] Whisper STT tiếng Việt             │
│  • Odometry bánh xe (50 Hz)                  │   ◄──────────────           │  • [Phase 2] Qwen2.5-14B lập kế hoạch          │
│  • EKF IMU+SLAM (100 Hz)                     │                             │  • [Phase 2] Piper TTS phản hồi giọng nói       │
│  • Camera stereo → bản đồ chiếm dụng        │   Lệnh hành động            │                                                  │
│  • Camera 4K UGREEN → stream lên server     │   ◄──────────────           │  Port 8090 — FastAPI inference server            │
│  • Web dashboard (port 5000)                 │                             │                                                  │
└──────────────────────────────────────────────┘                             └──────────────────────────────────────────────────┘
```

**Nguyên tắc thiết kế cốt lõi:** Robot vẫn hoạt động an toàn hoàn toàn khi mất kết nối server. Server chỉ là lớp nâng cao, không phải điều kiện bắt buộc cho an toàn.

---

## Cấu Hình Phần Cứng

### Jetson Nano (Robot)

#### Nền Tảng Tính Toán

| Thành phần | Thông số |
|---|---|
| **Bo mạch** | NVIDIA Jetson Nano 4GB |
| **GPIO** | Jetson.GPIO (điều khiển trực tiếp chân header) |
| **Bus I2C** | Bus 1 (SDA: pin 3, SCL: pin 5) |

#### Hệ Thống Di Chuyển

| Thành phần | Thông số |
|---|---|
| **Loại** | Bánh xích vi sai (tank-style) |
| **Động cơ** | 2× động cơ bước |
| **Driver** | 2× A4988 microstepping |
| **Microstepping** | 1/16 bước |
| **Bước/vòng** | 200 × 16 = 3200 bước/vòng |
| **Đường kính đĩa xích** | 69.84 mm (từ CAD) |
| **Khoảng cách bánh** | 284.5 mm (tâm đến tâm) |
| **Tỉ số truyền** | 38/18 = 2.111 (giảm tốc 1 cấp) |
| **Tốc độ tuyến tính tối đa** | 0.3 m/s |
| **Tốc độ góc tối đa** | 1.5 rad/s |

#### Sơ Đồ Chân GPIO Động Cơ (Jetson Nano Board Header)

| Tín hiệu | Motor Trái | Motor Phải |
|---|---|---|
| STEP | 33 | 32 |
| DIR | 35 | 36 |
| ENABLE | 37 | 38 |
| MS1 | 29 | 13 |
| MS2 | 31 | 15 |
| MS3 | 11 | 16 |

#### Camera Stereo (Lập bản đồ & SLAM)

| Thành phần | Thông số |
|---|---|
| **Loại** | 2× USB camera |
| **Camera trái** | ID: 1 (`/dev/video1`) |
| **Camera phải** | ID: 0 (`/dev/video0`) |
| **Độ phân giải** | 640 × 480 px |
| **FPS** | 30 fps |
| **Baseline stereo** | 140 mm |
| **File hiệu chỉnh** | `data/calibration.npz` |
| **Điều khiển phơi sáng** | Thủ công (exposure: −5, gain: 55) |

#### Camera 4K UGREEN (Nhận diện vật thể)

| Thành phần | Thông số |
|---|---|
| **Loại** | USB webcam UGREEN 4K |
| **Device** | `/dev/video2` |
| **Độ phân giải ghi** | 3840 × 2160 (4K) |
| **FPS ghi** | 15 fps |
| **Stream lên server** | 1280 × 720, JPEG 85% |
| **FOV ngang** | ~90° |

#### IMU — MPU-6050

| Thành phần | Thông số |
|---|---|
| **Chip** | InvenSense MPU-6050 |
| **Giao tiếp** | I2C, địa chỉ 0x68, bus 1 |
| **Dải đo gyro** | ±250 °/s |
| **Dải đo gia tốc** | ±2 g |
| **Hiệu chỉnh** | Bias tĩnh khi khởi động (500 mẫu, robot phải đứng yên) |

---

### Server x99 (Inference)

| Thành phần | Thông số |
|---|---|
| **CPU** | 2× Intel Xeon E5-2680v4 (28 nhân / 56 luồng) |
| **RAM** | 32 GB |
| **GPU** | AMD Radeon MI50 16 GB HBM2 (ROCm gfx906) |
| **Kết nối** | WiFi LAN — IP: `192.168.2.10` |
| **Port inference** | 8090 (FastAPI + YOLOv11) |
| **VRAM hiện tại** | YOLOv11m ~0.5 GB / 16 GB |

#### Ngân Sách VRAM (Hiện tại & Dự kiến)

```
Hiện tại (Phase 1):
  YOLOv11m                ~0.5 GB  ✓ đang chạy
  ─────────────────────────────────────────────
  Còn trống              ~15.5 GB

Dự kiến (Phase 2):
  YOLOv11m                ~0.5 GB
  Whisper large-v3 (STT)  ~1.5 GB
  Qwen2.5-14B Q4_K_M      ~9.0 GB
  ─────────────────────────────────────────────
  Tổng dùng              ~11.0 GB  ✓ vừa MI50 16 GB
  Còn trống               ~5.0 GB  (KV cache)
```

---

## Kiến Trúc Phần Mềm

### Sơ Đồ Module

```
main.py  ── RobotSystem (orchestrator + watchdog)
│
├── [Robot] stereo_camera.py          — Thu ảnh, rectify, disparity, VisualOdometry
├── [Robot] mpu6050_imu.py            — Driver I2C MPU-6050 + ComplementaryFilter
├── [Robot] imu_fusion.py             — EKF [x,y,θ,vx,vy,ω] + FusedSLAMWrapper
├── [Robot] orb_slam3_wrapper.py      — ORB-SLAM3 bindings + SimpleVisualSLAM fallback
├── [Robot] obstacle_detector.py      — Phát hiện chướng ngại vật 5 vùng từ depth map
├── [Robot] autonomous_mapper.py      — OccupancyGrid + A* + frontier exploration + bản đồ ngữ nghĩa
├── [Robot] jetson_motor_controller.py— Điều khiển bước GPIO A4988 + odometry
├── [Robot] safety_controller.py      — SafetyVelocityController (gating an toàn)
├── [Robot] server_client.py          — Client VisionServerClient (4K cam → x99 YOLOv11)
└── [Robot] web_server.py             — Flask REST API + dashboard (port 5000)

server/
└── [x99]  inference_server.py        — FastAPI + YOLOv11 (port 8090)
```

### Luồng Dữ Liệu Chính

```
Camera stereo ──► rectify ──► disparity ──► ObstacleDetector ──► SafetyController ──► Motors
                                    │
                                    └──► SimpleVisualSLAM ──► EKF (IMU+Vision) ──► Pose
                                                                        │
                                                                        └──► OccupancyGrid ──► A* ──► AutonomousMapper

Camera 4K ──► VisionServerClient ──►[WiFi]──► YOLOv11 x99 ──► detections ──► SemanticMap
```

### Mô Tả Module Quan Trọng

#### `imu_fusion.py` — Bộ Lọc Kalman Mở Rộng
`ExtendedKalmanFilter` duy trì vector trạng thái **[x, y, θ, vx, vy, ω]**:
- **Bước dự đoán**: gyro Z từ IMU (100 Hz), `dt` tính từ `last_imu_time` (không bị ảnh hưởng bởi cập nhật thị giác)
- **Bước hiệu chỉnh**: pose từ visual SLAM (chỉ khi pose thực sự thay đổi — không gửi pose trùng lặp vào EKF)
- **Joseph stabilised covariance update**: đảm bảo ma trận P luôn đối xứng và xác định dương

#### `autonomous_mapper.py` — Bản Đồ Ngữ Nghĩa
Ngoài bản đồ chiếm dụng 2D, mapper còn lưu **`SemanticObject`** từ YOLO:
- Gắn tọa độ thế giới dựa trên pose robot + góc phương vị từ camera
- Hợp nhất các quan sát gần nhau (trong 0.5 m) bằng trung bình có trọng số
- API: `get_objects_near('chair', 3.0)` — tìm ghế gần nhất trong 3 m

#### `server_client.py` — Kết Nối Server
- Chạy hoàn toàn trong background thread, không bao giờ chặn robot
- Timeout 150 ms — nếu server không phản hồi, giữ nguyên detection cũ và đánh dấu `server_available = False`
- Tự động fallback: robot tiếp tục hoạt động với obstacle detection từ stereo

---

## Robot Hiện Tại Có Thể Làm Gì

### ✅ Đã Hoạt Động (Phase 1)

#### Nhận Thức Môi Trường
| Khả năng | Chi tiết |
|---|---|
| **Đo độ sâu stereo** | StereoSGBM cho depth map theo pixel, tầm tối đa 2 m |
| **Phát hiện chướng ngại vật** | 5 vùng (trái/trái-trước/giữa/phải-trước/phải), lịch sử 5 frame để lọc nhiễu |
| **Đo hướng IMU** | MPU-6050 @ 100 Hz, hiệu chỉnh bias khi khởi động |
| **Nhận diện vật thể AI** | Camera 4K UGREEN → YOLOv11 trên server x99 → nhãn + bounding box @ ~15 fps |
| **Bản đồ ngữ nghĩa** | Vật thể YOLO được gắn tọa độ thế giới và lưu theo nhãn (ghế, cửa, người...) |

#### Định Vị & Lập Bản Đồ
| Khả năng | Chi tiết |
|---|---|
| **Visual SLAM** | ORB feature tracking + Essential Matrix, ước tính pose 3D liên tục |
| **Kết hợp IMU + Visual** | EKF 6 trạng thái giảm drift so với SLAM thuần túy |
| **Bản đồ chiếm dụng** | Grid 200×200 ô, 5 cm/ô (phủ 10×10 m), lưu/tải file `.npz` |
| **Odometry bánh xe** | Tích phân bước đếm @ 50 Hz → dead-reckoning pose |

#### Điều Hướng
| Khả năng | Chi tiết |
|---|---|
| **Khám phá tự động** | Frontier-based: tìm biên giữa vùng tự do và vùng chưa biết |
| **Lập kế hoạch đường đi A\*** | Tìm đường ngắn nhất trên grid với inflation chướng ngại vật theo bán kính robot |
| **Tránh chướng ngại vật** | Điều chỉnh hoặc chặn lệnh vận tốc khi phát hiện vật cản |
| **Dừng khẩn cấp** | Dừng ngay khi chướng ngại vật < 15 cm |

#### Điều Khiển & Giám Sát
| Khả năng | Chi tiết |
|---|---|
| **Web dashboard** | Truy cập qua trình duyệt tại `http://<jetson-ip>:5000` |
| **Stream video stereo** | MJPEG live tại `/video_feed` |
| **Stream camera 4K** | MJPEG với bbox YOLO overlay tại `/object_feed` |
| **Teleoperation an toàn** | Điều khiển thủ công qua API, có safety gating chướng ngại vật |
| **Watchdog** | Tự dừng motor nếu vòng xử lý chính không hoạt động > 5 giây |
| **Bảo mật API** | Token xác thực `X-Robot-Token` cho tất cả endpoint điều khiển |

### 🔄 Đang Phát Triển (Phase 2)

| Khả năng | Trạng thái |
|---|---|
| **Nhận lệnh giọng nói tiếng Việt** | Whisper large-v3 STT → Qwen2.5-14B → JSON hành động |
| **Phản hồi bằng giọng nói** | Piper TTS tiếng Việt |
| **Điều hướng theo ngữ nghĩa** | "Đến chỗ cái ghế" → `get_objects_near('chair')` → A* navigate |
| **ORB-SLAM3 đầy đủ** | Cần biên dịch C++ binding trên Jetson, có loop closure |

---

## Cấu Trúc Dự Án

```
Ver2/
├── main.py                         # Điểm vào — RobotSystem orchestrator
├── config.yaml                     # Toàn bộ tham số phần cứng & điều chỉnh
│
├── [Robot modules]
├── autonomous_mapper.py            # OccupancyGrid, AutonomousMapper, A*, SemanticMap
├── imu_fusion.py                   # Extended Kalman Filter, FusedSLAMWrapper
├── jetson_motor_controller.py      # A4988 GPIO stepper, odometry
├── mpu6050_imu.py                  # Driver MPU-6050 I2C, ComplementaryFilter
├── obstacle_detector.py            # Phát hiện chướng ngại vật từ depth map
├── orb_slam3_wrapper.py            # ORB-SLAM3 + SimpleVisualSLAM fallback
├── safety_controller.py            # Gating vận tốc an toàn
├── server_client.py                # VisionServerClient (4K cam → x99 YOLOv11)
├── stereo_camera.py                # Thu ảnh stereo, rectify, disparity, VO
├── vision_debug_server.py          # Debug stream server
├── web_server.py                   # Flask dashboard + REST API (port 5000)
│
├── server/
│   └── inference_server.py         # FastAPI + YOLOv11 chạy trên x99 (port 8090)
│
├── data/
│   ├── calibration.npz             # Dữ liệu hiệu chỉnh camera stereo
│   └── room_map.npz                # Bản đồ chiếm dụng đã lưu
│
├── docs/                           # Tài liệu hướng dẫn phần cứng
├── logs/                           # File log runtime
├── templates/
│   ├── index.html                  # Dashboard web chính
│   └── vision_debug.html           # Giao diện debug vision
├── tests/                          # Unit test và integration test
│   ├── imu/
│   ├── motors/
│   └── sensors/
└── tools/                          # Tiện ích hiệu chỉnh và debug
    ├── calibration/
    ├── camera/
    └── debug/
```

---

## Cấu Hình config.yaml

Tất cả tham số phần cứng trong [`config.yaml`](config.yaml). **Không cần sửa Python** để điều chỉnh phần cứng.

```yaml
# Camera stereo (SLAM + obstacle detection)
camera:
  left_id: 1
  right_id: 0
  stereo_baseline: 0.14       # mét — phải khớp phần cứng thực tế
  width: 640
  height: 480
  fps: 30
  use_manual_settings: true   # tắt auto-exposure để stereo nhất quán
  manual_exposure: -5
  manual_gain: 55

# Camera 4K UGREEN (object detection)
object_camera:
  device_id: 2                # /dev/video2
  capture_width: 3840
  capture_height: 2160
  capture_fps: 15
  stream_width: 1280          # downscale trước khi gửi lên server
  stream_height: 720
  jpeg_quality: 85

# x99 inference server
vision_server:
  host: "192.168.2.10"
  port: 8090
  enabled: true
  timeout_s: 0.15             # robot không bao giờ chờ quá 150ms
  min_confidence: 0.40
  camera_hfov_deg: 90.0       # FOV ngang UGREEN webcam

# Động cơ
motor:
  wheel_diameter: 0.06984     # mét (69.84 mm đường kính đĩa xích)
  wheel_base: 0.2845          # mét (284.5 mm khoảng bánh)
  gear_ratio: 2.111           # 38/18 (giảm tốc 1 cấp)
  microsteps: 16
  max_linear_speed: 0.3       # m/s
  max_angular_speed: 1.5      # rad/s

# Lập bản đồ
mapping:
  grid_width: 10.0            # mét
  grid_height: 10.0           # mét
  resolution: 0.05            # mét/ô

# Phát hiện chướng ngại vật
obstacle_detection:
  min_safe_distance: 0.3      # mét
  critical_distance: 0.15     # mét — dừng khẩn cấp

# IMU & EKF
imu:
  bus_number: 1
  address: 0x68
  use_ekf: true
  fusion_weight: 0.3

# SLAM
slam:
  use_orbslam3: false         # bật khi đã biên dịch C++ bindings
```

---

## Hướng Dẫn Khởi Động

### 1. Cài Đặt (Jetson Nano)

```bash
pip install flask opencv-python numpy smbus2 pyyaml
pip install Jetson.GPIO   # chỉ trên Jetson
```

### 2. Cài Đặt (Server x99)

```bash
# PyTorch với ROCm 5.6 cho MI50 (gfx906)
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.6
pip install ultralytics fastapi uvicorn opencv-python-headless numpy

# Biến môi trường bắt buộc cho MI50
export HSA_OVERRIDE_GFX_VERSION=9.0.6
```

### 3. Khởi Động Server x99

```bash
# Trên máy x99 (192.168.2.10):
export HSA_OVERRIDE_GFX_VERSION=9.0.6
python3 server/inference_server.py --model yolo11m.pt --port 8090

# Kiểm tra server hoạt động:
curl http://192.168.2.10:8090/health
```

### 4. Hiệu Chỉnh Camera Stereo (lần đầu)

```bash
python tools/calibration/calibrate_motors.py
# Dữ liệu lưu vào data/calibration.npz
```

### 5. Bảo Mật API (khuyến nghị)

```bash
# Trên Jetson, tạo token ngẫu nhiên:
export ROBOT_API_TOKEN=$(python3 -c "import secrets; print(secrets.token_hex(32))")
# Thêm vào ~/.bashrc để giữ qua reboot
```

### 6. Khởi Động Robot

```bash
python main.py
```

> **Lưu ý:** Giữ robot **đứng yên** trong 2-3 giây đầu khi khởi động để IMU hiệu chỉnh bias (500 mẫu).

Hệ thống khởi tạo theo thứ tự:
1. Camera stereo → 2. IMU → 3. SLAM → 4. Motor → 5. Obstacle detector → 6. Occupancy grid → 7. Autonomous mapper → 8. Camera 4K + Vision client → Web server

---

## Giao Diện Web

Truy cập: **`http://<jetson-ip>:5000`**

### Endpoints REST API

| Endpoint | Phương thức | Mô tả |
|---|---|---|
| `GET /` | — | Dashboard chính |
| `GET /api/status` | — | Trạng thái hệ thống (motor, SLAM, mapper, IMU) |
| `GET /api/pose` | — | Pose hiện tại `{x, y, theta}` |
| `GET /api/imu` | — | Dữ liệu IMU (accel, gyro, orientation, nhiệt độ) |
| `GET /api/detections` | — | YOLO detections mới nhất từ server x99 |
| `GET /api/semantic_map` | — | Bản đồ ngữ nghĩa (vị trí thế giới các vật thể) |
| `GET /video_feed` | — | MJPEG stream camera stereo |
| `GET /object_feed` | — | MJPEG stream 4K với bbox YOLO overlay |
| `GET /map_feed` | — | MJPEG stream bản đồ chiếm dụng live |
| `POST /api/motor/velocity` | 🔒 Token | Đặt vận tốc `{linear, angular}` |
| `POST /api/motor/stop` | 🔒 Token | Dừng khẩn cấp |
| `POST /api/motor/enable` | 🔒 Token | Kích hoạt động cơ |
| `POST /api/motor/disable` | 🔒 Token | Tắt động cơ |
| `POST /api/exploration/start` | 🔒 Token | Bắt đầu khám phá tự động |
| `POST /api/exploration/stop` | 🔒 Token | Dừng khám phá |
| `POST /api/map/save` | 🔒 Token | Lưu bản đồ hiện tại |
| `POST /api/map/load` | 🔒 Token | Tải bản đồ từ file |

🔒 = Yêu cầu header `X-Robot-Token: <token>`

---

## Hạn Chế Hiện Tại & Kế Hoạch Nâng Cấp

### Hạn Chế Kỹ Thuật

| Vấn đề | Mô tả | Giải pháp |
|---|---|---|
| **Drift vị trí** | EKF chỉ dùng gyro Z để dự đoán; `vx/vy` không có hiệu chỉnh từ odometry bánh xe | Thêm `update_from_odometry()` vào EKF |
| **Không có loop closure** | `SimpleVisualSLAM` tích lũy sai số, không nhận ra nơi đã đến | Bật ORB-SLAM3 hoặc thêm bag-of-words |
| **Bản đồ không lưu tự động** | Cần gọi API `/api/map/save` thủ công | Thêm auto-save mỗi N frontiers |
| **StereoSGBM chậm trên Nano** | ~50-150ms/frame, thất bại ở tường không có họa tiết | Offload sang RAFT-Stereo trên x99 |
| **Độ chính xác pulse bước** | Python GIL gây jitter timing ở tốc độ cao | Dùng `pigpio` hardware PWM |
| **Không có giám sát pin** | Không biết mức pin robot | Thêm INA219 + endpoint `/api/battery` |

### Kế Hoạch Nâng Cấp

```
Phase 1 (Hiện tại) ── Lập bản đồ tự động + nhận diện vật thể YOLOv11
      │
      ▼
Phase 2 (Tiếp theo) ── Trợ lý giọng nói tiếng Việt
      │   • Whisper large-v3 STT (tiếng Việt)
      │   • Qwen2.5-14B lập kế hoạch hành động → JSON
      │   • Piper TTS phản hồi bằng giọng nói
      │   • Điều hướng theo tên vật thể: "Đến chỗ cái ghế"
      │
      ▼
Phase 3 (Tương lai) ── SLAM nâng cao + bản đồ ngữ nghĩa 3D
          • ORB-SLAM3 chạy trên x99 (loop closure đầy đủ)
          • RAFT-Stereo neural depth thay StereoSGBM
          • Bản đồ ngữ nghĩa 3D (vật thể + tọa độ)
```
