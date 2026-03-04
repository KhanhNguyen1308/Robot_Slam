# SƯỜN SLIDE THUYẾT TRÌNH
## ROBOT TỰ HÀNH LẬP BẢN ĐỒ THƯ VIỆN

---

## 📋 HƯỚNG DẪN SỬ DỤNG
- Tài liệu này là **sườn nội dung** cho slide thuyết trình
- Mỗi phần đánh dấu `---` là một slide riêng
- Có thể export sang PowerPoint, Google Slides, hoặc sử dụng Reveal.js
- Thời gian thuyết trình dự kiến: **20-30 phút**
- Bao gồm cả demo/video nếu có

---

---

# SLIDE 1: TRANG BÌA

## ROBOT TỰ HÀNH LẬP BẢN ĐỒ THƯ VIỆN
### Autonomous Library Mapping Robot

**Phiên bản:** Ver 2.0

**Nhóm thực hiện:** [Tên nhóm]

**Giảng viên hướng dẫn:** [Tên GVHD]

**Ngày:** [Ngày tháng năm]

---

**Hình ảnh:** Ảnh robot thực tế (nếu có) hoặc sơ đồ 3D

---

---

# SLIDE 2: MỤC TIÊU DỰ ÁN

## 🎯 Mục tiêu

### Mục tiêu chính:
- Xây dựng robot di động tự động lập bản đồ thư viện
- Tích hợp công nghệ SLAM (Simultaneous Localization and Mapping)
- Tự động phát hiện và tránh vật cản

### Ứng dụng thực tế:
- 📚 Lập bản đồ tự động cho thư viện
- 🚨 Hệ thống tuần tra an ninh
- 📦 Quản lý kho hàng
- 🔬 Nền tảng nghiên cứu robotics

---

---

# SLIDE 3: TỔNG QUAN HỆ THỐNG

## 🤖 Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────┐
│           JETSON NANO (Brain)                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐     │
│  │  SLAM    │  │ Planning │  │  Control │     │
│  └──────────┘  └──────────┘  └──────────┘     │
└──────┬──────────────┬──────────────┬───────────┘
       │              │              │
   ┌───▼────┐    ┌────▼────┐    ┌───▼────┐
   │Stereo  │    │ MPU6050 │    │ Motors │
   │Camera  │    │   IMU   │    │ A4988  │
   └────────┘    └─────────┘    └────────┘
```

### Thành phần chính:
- **Điều khiển:** NVIDIA Jetson Nano
- **Cảm biến:** Stereo Camera + IMU
- **Truyền động:** 2× NEMA 17 Stepper Motors
- **Giao diện:** Web Dashboard

---

---

# SLIDE 4: PHẦN CỨNG - BỘ XỬ LÝ

## 💻 NVIDIA Jetson Nano

### Thông số kỹ thuật:
| Thông số | Giá trị |
|----------|---------|
| CPU | Quad-core ARM Cortex-A57 @ 1.43 GHz |
| GPU | 128-core NVIDIA Maxwell |
| RAM | 4GB LPDDR4 |
| CUDA | Có (10.2) |
| Nguồn | 5V 4A |

### Lý do chọn:
✅ Tích hợp GPU → Xử lý Computer Vision nhanh  
✅ Hỗ trợ CUDA → Deep Learning  
✅ GPIO pins → Điều khiển phần cứng  
✅ Giá cả phải chăng (~$100)

---

**Hình ảnh:** Jetson Nano board

---

---

# SLIDE 5: PHẦN CỨNG - HỆ THỐNG THỊ GIÁC

## 📷 Stereo Camera System

### Cấu hình:
- **Loại:** 2× USB Camera
- **Độ phân giải:** 640×480 pixels
- **Frame rate:** 30 FPS
- **Baseline:** 140mm (khoảng cách giữa 2 camera)

### Chức năng:
1. **Depth Estimation** - Ước tính độ sâu
2. **Visual Odometry** - Định vị từ thị giác
3. **Obstacle Detection** - Phát hiện vật cản
4. **3D Reconstruction** - Tạo bản đồ 3D

---

**Công thức tính độ sâu:**
$$Z = \frac{f \cdot B}{d}$$
- $f$: Tiêu cự camera
- $B$: Baseline (14cm)
- $d$: Disparity (độ chênh pixel)

---

**Hình ảnh:** Stereo camera setup, ảnh disparity map

---

---

# SLIDE 6: PHẦN CỨNG - CẢM BIẾN QUÁN TÍNH

## 🧭 MPU6050 IMU

### Thông số:
| Cảm biến | Thông số |
|----------|----------|
| Accelerometer | ±2g |
| Gyroscope | ±250°/s |
| Interface | I²C (0x68) |
| Sample Rate | 100 Hz |

### Vai trò:
- **Đo góc quay** (yaw, pitch, roll)
- **Đo gia tốc** tuyến tính
- **Kết hợp với camera** → Tăng độ chính xác SLAM

### Sensor Fusion:
```
Camera (30Hz) + IMU (100Hz) 
    ↓
Extended Kalman Filter
    ↓
Pose chính xác hơn
```

---

**Hình ảnh:** MPU6050 module

---

---

# SLIDE 7: PHẦN CỨNG - HỆ THỐNG TRUYỀN ĐỘNG

## ⚙️ Motor & Drive System

### Motor:
- **Loại:** NEMA 17 Stepper Motor
- **Số lượng:** 2 motors (trái + phải)
- **Bước/vòng:** 200 steps (1.8°/step)
- **Microstepping:** 1/16 step → 3200 steps/vòng
- **Driver:** A4988

### Bánh xe:
- **Đường kính:** 66mm
- **Wheelbase:** 275mm
- **Tỷ số truyền:** 2.111:1 (belt drive)

### Kinematics:
$$v_L = v - \frac{\omega L}{2}, \quad v_R = v + \frac{\omega L}{2}$$

---

**Hình ảnh:** NEMA 17, A4988 driver, sơ đồ differential drive

---

---

# SLIDE 8: SƠ ĐỒ KẾT NỐI ĐIỆN

## 🔌 Electrical Wiring

```
Battery 12V
   ├── Buck 5V → Jetson Nano
   ├── A4988 Left → Motor Left
   └── A4988 Right → Motor Right

Jetson GPIO:
   ├── Pin 33,35,37 → A4988 Left (STEP,DIR,EN)
   ├── Pin 32,36,38 → A4988 Right
   ├── Pin 3,5 → MPU6050 (SDA,SCL)
   └── USB 0,1 → Cameras
```

### Safety:
- ⚠️ **GND chung** giữa Jetson và A4988
- ⚠️ **VREF = 0.4V** cho motor 1.5A
- ⚠️ **Tụ 100μF** gần VMOT

---

**Hình ảnh:** Wiring diagram, ảnh thực tế

---

---

# SLIDE 9: SLAM - KHÁI NIỆM

## 🗺️ SLAM là gì?

### Simultaneous Localization and Mapping
= **Định vị đồng thời và lập bản đồ**

### Bài toán:
- Robot **không biết** vị trí của mình
- Robot **không biết** bản đồ môi trường
- Phải **cùng lúc** tìm ra cả hai!

### Ví dụ:
> Bạn đi vào một căn phòng tối, không biết mình đang đứng đâu, 
> cũng không biết căn phòng trông như thế nào.
> SLAM giúp bạn vừa đi vừa vẽ bản đồ, vừa biết mình đang ở đâu.

---

**Hình ảnh:** Minh họa SLAM concept

---

---

# SLIDE 10: VISUAL SLAM

## 👁️ Visual SLAM Pipeline

```
┌──────────┐     ┌──────────────┐     ┌──────────┐
│  Stereo  │────▶│   Feature    │────▶│  Pose    │
│  Images  │     │  Tracking    │     │Recovery  │
└──────────┘     └──────────────┘     └──────────┘
                                            │
                                            ▼
                                      ┌──────────┐
                                      │  Map     │
                                      │  Points  │
                                      └──────────┘
```

### Các bước:
1. **Feature Detection:** ORB features (1000-2000 points)
2. **Feature Matching:** Tìm điểm tương ứng giữa 2 frame
3. **Essential Matrix:** Tính rotation & translation
4. **Triangulation:** Tính tọa độ 3D của điểm
5. **Pose Update:** Cập nhật vị trí robot

---

**Hình ảnh:** Feature matching visualization

---

---

# SLIDE 11: SENSOR FUSION - EKF

## 🔀 Extended Kalman Filter

### State Vector:
$$\mathbf{x} = [x, y, \theta, v_x, v_y, \omega]^T$$

### Quy trình:
```
┌─────────────┐
│  Prediction │ ← IMU Gyroscope (100Hz)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Update    │ ← Visual Odometry (30Hz)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  Fused Pose │ (Chính xác hơn)
└─────────────┘
```

### Lợi ích:
- ✅ Giảm drift của visual odometry
- ✅ Tăng tần suất cập nhật (100Hz thay vì 30Hz)
- ✅ Cải thiện độ chính xác góc quay

---

---

# SLIDE 12: OCCUPANCY GRID MAPPING

## 🗺️ Bản đồ 2D

### Occupancy Grid:
- **Kích thước:** 10m × 10m
- **Độ phân giải:** 5cm/cell → 200×200 cells
- **Giá trị cell:**
  - `-1`: Chưa khám phá (màu xám)
  - `0`: Không gian trống (màu trắng)
  - `100`: Vật cản (màu đen)

### Ray Tracing:
- Sử dụng **Bresenham algorithm**
- Từ vị trí robot → điểm vật cản:
  - Tất cả cell trên đường đi = FREE
  - Cell cuối cùng = OCCUPIED

---

**Hình ảnh:** Occupancy grid visualization

---

---

# SLIDE 13: AUTONOMOUS EXPLORATION

## 🚀 Khám phá tự động

### Chiến lược: Frontier-Based Exploration

```
1. Phát hiện Frontiers 
   (biên giới giữa vùng đã biết và chưa biết)
   
2. Chọn Frontier gần nhất
   
3. Lập kế hoạch đường đi
   
4. Di chuyển đến Frontier
   
5. Lặp lại cho đến khi khám phá hết
```

### Frontier là gì?
> Cell **trống** có ít nhất 1 neighbor **chưa khám phá**

---

**Hình ảnh:** Frontier detection visualization

---

---

# SLIDE 14: OBSTACLE DETECTION

## 🚧 Phát hiện vật cản

### Phân vùng phát hiện (5 zones):

```
        LEFT      FRONT_LEFT     CENTER    FRONT_RIGHT    RIGHT
         │             │            │           │           │
    ─────┴─────────────┴────────────┴───────────┴───────────┴─────
   -90°        -45°          0°           45°              90°
   
   Khoảng cách an toàn:
   - Front zones: 30cm
   - Side zones: 21cm
   - Critical: 15cm
```

### Hành động:
| Tình huống | Hành động |
|------------|-----------|
| Vật cản < 15cm | DỪNG KHẨN CẤP |
| Vật cản 15-30cm | Giảm tốc + Rẽ |
| Vật cản > 30cm | Tiếp tục |

---

**Hình ảnh:** Obstacle zones diagram, depth map

---

---

# SLIDE 15: WEB DASHBOARD

## 🖥️ Giao diện giám sát

### Tính năng:
- 📹 **Live Video:** Camera stereo real-time
- 🗺️ **Depth Map:** Bản đồ độ sâu
- 📊 **Occupancy Grid:** Bản đồ 2D
- 📈 **SLAM Status:** Trạng thái tracking
- 🧭 **IMU Data:** Góc quay, gia tốc
- ⌨️ **Manual Control:** Điều khiển WASD
- 🤖 **Auto Mode:** Bật/tắt tự động

### Công nghệ:
- Backend: **Flask** (Python)
- Frontend: **HTML5/JavaScript/Canvas**
- Realtime: **WebSocket**

---

**Hình ảnh:** Screenshot web dashboard

---

---

# SLIDE 16: KINEMATICS CALCULATION

## 📐 Tính toán động học

### Inverse Kinematics:
Cho vận tốc mong muốn $v$ (m/s) và $\omega$ (rad/s):

$$v_L = v - \frac{\omega \cdot L}{2}$$
$$v_R = v + \frac{\omega \cdot L}{2}$$

### Chuyển đổi sang steps/s:

$$\text{steps/s} = v \times \text{steps\_per\_meter}$$

**Với:**
- Steps/rev = 200 × 16 × 2.111 = 6755 steps
- Wheel circumference = π × 0.066 = 0.207m
- Steps/meter = 6755 / 0.207 = **32,589 steps/m**

### Ví dụ:
$v = 0.1$ m/s → $3259$ steps/s

---

---

# SLIDE 17: DEPTH CALCULATION

## 📏 Tính toán độ sâu

### Công thức Stereo Vision:

$$Z = \frac{f \cdot B}{d}$$

**Trong đó:**
- $Z$: Độ sâu (khoảng cách) [meters]
- $f$: Tiêu cự camera [pixels] ~ 500px
- $B$: Baseline [meters] = 0.14m
- $d$: Disparity [pixels]

### Ví dụ tính toán:
| Disparity | Depth |
|-----------|-------|
| 50 px | 1.4 m |
| 100 px | 0.7 m |
| 200 px | 0.35 m |

**Lưu ý:** Độ chính xác giảm theo khoảng cách

---

---

# SLIDE 18: THÔNG SỐ HIỆU NĂNG

## ⚡ Hiệu năng hệ thống

### Processing Speed:
- **Visual SLAM:** 10-15 FPS
- **IMU Fusion:** 100 Hz
- **Obstacle Detection:** 15-20 FPS
- **Motor Control:** 1000 Hz (steps)

### Độ chính xác:
- **Position:** ±5cm (điều kiện tốt)
- **Orientation:** ±2° (với IMU)
- **Depth:** ±10% ở khoảng cách 1-2m

### Tốc độ di chuyển:
- **Max linear:** 0.3 m/s (~1 km/h)
- **Max angular:** 1.5 rad/s (~86°/s)

### Thời gian hoạt động:
- **Battery:** Li-Po 5000mAh
- **Runtime:** 2-3 giờ

---

---

# SLIDE 19: KẾT QUẢ THỰC NGHIỆM

## 📊 Kết quả đạt được

### Tính năng hoàn thành:
- ✅ SLAM đa phương thức (Camera + IMU)
- ✅ Lập bản đồ 2D tự động
- ✅ Phát hiện và tránh vật cản
- ✅ Khám phá tự động (Frontier-based)
- ✅ Web dashboard điều khiển
- ✅ Sensor fusion (EKF)

### Thử nghiệm:
- 🏢 **Môi trường:** Phòng/hành lang thư viện
- 📐 **Diện tích map:** 5×5m đến 8×8m
- ⏱️ **Thời gian:** 10-15 phút khám phá
- 🎯 **Độ chính xác:** Map tương đồng 85-90% với thực tế

---

**Hình ảnh/Video:** Kết quả map thực tế, video demo

---

---

# SLIDE 20: ĐIỂM YẾU & HẠN CHẾ

## ⚠️ Hạn chế hiện tại

### Phần cứng:
- ❌ USB Camera → Không đồng bộ hardware
- ❌ Motor không có encoder → Không feedback
- ❌ Battery life ngắn (2-3h)

### Thuật toán:
- ❌ Simple SLAM → Không có loop closure
- ❌ Drift tích lũy theo thời gian
- ❌ Frontier exploration chưa tối ưu global

### Hiệu năng:
- ❌ Processing speed thấp (10-15 FPS)
- ❌ Không thể di chuyển nhanh
- ❌ Giới hạn diện tích map (~100m²)

---

---

# SLIDE 21: CẢI TIẾN TƯƠNG LAI (1)

## 🚀 Nâng cấp phần cứng

### Ưu tiên cao:

1. **DC Motor + Encoder**
   - Closed-loop control
   - Odometry chính xác
   - Phát hiện trượt bánh

2. **IMX219 Stereo Camera**
   - Hardware trigger → Đồng bộ perfect
   - CSI interface → Latency thấp
   - CUDA pipeline

3. **LIDAR 2D (RPLIDAR A1)**
   - 360° coverage
   - Không phụ thuộc ánh sáng
   - Hybrid SLAM (Vision + LIDAR)

---

---

# SLIDE 22: CẢI TIẾN TƯƠNG LAI (2)

## 🧠 Nâng cấp thuật toán

### Phase 1: SLAM
- **ORB-SLAM3 full:** Loop closure + relocalization
- **g2o optimization:** Graph-based SLAM
- **Map persistence:** Lưu và tải lại map

### Phase 2: Planning
- **A* Path Planning:** Đường đi tối ưu
- **Dynamic replanning:** Cập nhật khi có vật cản mới
- **Trajectory smoothing:** Đường đi mượt mà

### Phase 3: AI
- **Object Detection:** YOLOv5-nano
- **Semantic SLAM:** Hiểu ngữ nghĩa môi trường
- **Human detection:** Tránh va chạm người

---

---

# SLIDE 23: ROS INTEGRATION

## 🤖 Tích hợp ROS

### Robot Operating System

**Lợi ích:**
- ✅ Standardized framework
- ✅ Reuse packages (gmapping, navigation)
- ✅ Simulation (Gazebo)
- ✅ Visualization (RViz)
- ✅ Multi-robot coordination

### Architecture:
```
/camera_node → /slam_node → /map
                    ↓
              /path_planner
                    ↓
              /navigation → /motor_controller
                    ↑
            /obstacle_detector
```

---

**Hình ảnh:** ROS architecture diagram

---

---

# SLIDE 24: TÍNH NĂNG MỞ RỘNG

## 🌟 Tính năng tương lai

### 1. Autonomous Charging
- Tự tìm dock sạc
- Automatic docking
- Unlimited runtime

### 2. Cloud Integration
- Upload map/logs
- Remote monitoring
- Fleet management

### 3. Voice Control
- "Go to section A"
- "Return home"
- Speech-to-text

### 4. Multi-Robot Collaboration
- Distributed SLAM
- Task allocation
- Map sharing

---

---

# SLIDE 25: ROADMAP PHÁT TRIỂN

## 🗓️ Kế hoạch 12 tháng

| Phase | Thời gian | Nội dung |
|-------|-----------|----------|
| **Phase 1** | 1-2 tháng | Ổn định hệ thống, fix bugs |
| **Phase 2** | 2-3 tháng | Nâng cấp cảm biến (encoder, LIDAR) |
| **Phase 3** | 3-4 tháng | ORB-SLAM3, A*, ML |
| **Phase 4** | 2-3 tháng | ROS integration |
| **Phase 5** | 3-6 tháng | Tính năng nâng cao |

**Mục tiêu 1 năm:** Hệ thống robot hoàn chỉnh với ROS, multi-sensor, AI

---

---

# SLIDE 26: BÀI HỌC KINH NGHIỆM

## 💡 Lessons Learned

### Thành công:
- ✅ Sensor fusion cải thiện SLAM đáng kể
- ✅ Web dashboard rất hữu ích cho debug
- ✅ Modular code → Dễ mở rộng
- ✅ Threading architecture → Performance tốt

### Khó khăn:
- ⚠️ Camera calibration rất quan trọng và khó
- ⚠️ Lighting conditions ảnh hưởng lớn
- ⚠️ Stepper motor skip steps khi tải nặng
- ⚠️ Power management phức tạp

### Khuyến nghị:
- 💬 Đầu tư thời gian cho calibration
- 💬 Test nhiều trong điều kiện thực tế
- 💬 Sử dụng encoder từ đầu
- 💬 Simulation trước khi làm hardware

---

---

# SLIDE 27: DEMO

## 🎬 DEMO THỰC TẾ

### Nội dung demo:

1. **System startup**
   - Khởi động robot
   - Hiển thị web dashboard

2. **Manual control**
   - Điều khiển bằng bàn phím
   - Quan sát SLAM tracking

3. **Autonomous exploration**
   - Bật chế độ tự động
   - Robot tự khám phá
   - Hiển thị map tạo ra

4. **Obstacle avoidance**
   - Đặt vật cản
   - Robot tự động tránh

---

**[Chạy video demo hoặc demo live nếu có]**

---

---

# SLIDE 28: TÀI LIỆU THAM KHẢO

## 📚 References

### Papers:
1. ORB-SLAM3: Campos et al., 2021
2. Stereo Vision: Hartley & Zisserman, "Multiple View Geometry"
3. EKF SLAM: Thrun et al., "Probabilistic Robotics"

### Code/Libraries:
- OpenCV: https://opencv.org
- Jetson GPIO: https://github.com/NVIDIA/jetson-gpio
- Flask: https://flask.palletsprojects.com

### Hardware:
- NVIDIA Jetson: https://developer.nvidia.com/jetson
- A4988 Datasheet: Allegro Microsystems
- MPU6050 Datasheet: InvenSense

---

---

# SLIDE 29: KẾT LUẬN

## 🎓 Kết luận

### Đạt được:
- ✅ Xây dựng thành công robot SLAM tự hành
- ✅ Tích hợp đa cảm biến (Camera + IMU)
- ✅ Khám phá và lập bản đồ tự động
- ✅ Hệ thống hoạt động ổn định

### Ý nghĩa:
- 🔬 **Học thuật:** Áp dụng lý thuyết SLAM, sensor fusion
- 💼 **Thực tiễn:** Giải quyết bài toán mapping thực tế
- 🚀 **Tương lai:** Nền tảng cho robot service

### Triển vọng:
- Thương mại hóa cho thư viện, kho
- Nền tảng nghiên cứu robotics
- Mở rộng sang multi-robot system

---

---

# SLIDE 30: Q&A

## ❓ HỎI & ĐÁP

### Cảm ơn đã theo dõi!

---

**Liên hệ:**
- Email: [your-email]
- GitHub: [repository-link]
- Documentation: THUYET_MINH_KY_THUAT.md

---

**Sẵn sàng trả lời câu hỏi!**

---

---

# PHỤ LỤC: BACKUP SLIDES

## 📎 Các slide dự phòng cho Q&A

---

---

# BACKUP 1: CHI TIẾT SENSOR FUSION

## Extended Kalman Filter

### Prediction step:
$$\hat{x}_{k+1} = f(\hat{x}_k, u_k)$$
$$P_{k+1} = F_k P_k F_k^T + Q_k$$

### Update step:
$$K = P H^T (H P H^T + R)^{-1}$$
$$\hat{x}_{new} = \hat{x} + K(z - H\hat{x})$$
$$P_{new} = (I - KH)P$$

**Measurement:**
- Visual: $(x, y, \theta)$ @ 30Hz
- IMU: $\omega$ @ 100Hz

---

---

# BACKUP 2: A4988 VREF CALCULATION

## VREF Adjustment

$$V_{REF} = I_{max} \times 8 \times R_{sense}$$

**Với $R_{sense} = 0.05\Omega$:**
$$V_{REF} = I_{max} \times 0.4$$

**Motor 1.5A:**
- Max: $V_{REF} = 0.6V$
- Recommended (70%): $V_{REF} = 0.4V$

**Đo bằng multimeter:**
- DC Voltage mode
- Đo giữa POT wiper và GND
- Vặn POT để điều chỉnh

---

---

# BACKUP 3: CAMERA CALIBRATION

## Stereo Calibration Process

1. **Chuẩn bị:** Checkerboard 9×6
2. **Chụp:** 20-30 cặp ảnh
3. **Detect:** Tìm corners
4. **Calibrate:** Tính K, D, R, T
5. **Save:** .npz file
6. **Verify:** Kiểm tra epipolar lines

**Tiêu chí đánh giá:**
- Reprojection error < 0.5 pixels
- Epipolar lines ngang
- RMS error < 1.0

---

---

# BACKUP 4: COMPARISON TABLE

## So sánh các phương pháp SLAM

| Method | Type | Accuracy | Speed | Cost |
|--------|------|----------|-------|------|
| Visual SLAM | Camera | Medium | Fast | Low |
| LIDAR SLAM | LIDAR | High | Fast | High |
| VO + IMU | Hybrid | High | Medium | Low |
| ORB-SLAM3 | Camera | Very High | Slow | Low |

**Lựa chọn của dự án:**
- Simple Visual + IMU → Hiện tại
- ORB-SLAM3 + IMU → Tương lai
- Hybrid (Visual + LIDAR + IMU) → Mục tiêu cuối

---

---

# BACKUP 5: COST BREAKDOWN

## Chi phí dự án

| Hạng mục | Đơn giá (VNĐ) | Số lượng | Thành tiền |
|----------|---------------|----------|------------|
| Jetson Nano 4GB | 2,500,000 | 1 | 2,500,000 |
| Stereo Camera | 500,000 | 2 | 1,000,000 |
| MPU6050 | 50,000 | 1 | 50,000 |
| NEMA 17 | 200,000 | 2 | 400,000 |
| A4988 Driver | 50,000 | 2 | 100,000 |
| Battery Li-Po | 500,000 | 1 | 500,000 |
| Chassis + Parts | 500,000 | 1 | 500,000 |
| **Tổng** | | | **~5,000,000** |

*Giá tham khảo, có thể thay đổi*

---

---

# END OF SLIDES

**Tổng số slide:** 30 slides chính + 5 backup slides

**Thời gian:** ~25-30 phút (có demo)

**Format:** Markdown → Có thể convert sang:
- PowerPoint (.pptx)
- Google Slides
- Reveal.js (HTML presentation)
- PDF

---
