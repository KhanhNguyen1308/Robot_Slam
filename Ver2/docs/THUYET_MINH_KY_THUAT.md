# THUYẾT MINH KỸ THUẬT
## ROBOT TỰ HÀNH LẬP BẢN ĐỒ THƯVIỆN

---

## 📋 THÔNG TIN DỰ ÁN

**Tên dự án:** Robot tự hành lập bản đồ và tuần tra thư viện  
**Phiên bản:** Ver 2.0  
**Nền tảng phần cứng:** NVIDIA Jetson Nano  
**Ngôn ngữ lập trình:** Python 3  
**Loại robot:** Differential Drive Mobile Robot  

---

## 1. TỔNG QUAN HỆ THỐNG

### 1.1. Giới thiệu
Robot tự hành lập bản đồ thư viện là một hệ thống robot di động tự động, được thiết kế để tự động khám phá, lập bản đồ và tuần tra không gian thư viện. Robot sử dụng công nghệ SLAM (Simultaneous Localization and Mapping) kết hợp với hệ thống cảm biến đa phương thức để định vị chính xác và tạo bản đồ 2D của môi trường xung quanh.

### 1.2. Mục tiêu
- Tự động khám phá và lập bản đồ môi trường thư viện
- Định vị chính xác vị trí robot trong không gian
- Phát hiện và tránh vật cản trong thời gian thực
- Cung cấp giao diện web để giám sát và điều khiển từ xa
- Tích hợp nhiều nguồn cảm biến để tăng độ chính xác

### 1.3. Ứng dụng
- Lập bản đồ tự động cho thư viện, kho sách
- Hệ thống tuần tra tự động
- Nền tảng nghiên cứu SLAM và robotics
- Giám sát và quản lý không gian trong nhà

---

## 2. TÍNH NĂNG KỸ CHIẾN THUẬT

### 2.1. Tính năng chính

#### 2.1.1. SLAM (Simultaneous Localization and Mapping)
**Mô tả:** Hệ thống SLAM đa phương thức kết hợp thị giác stereo và IMU

**Khả năng:**
- **Visual SLAM:** Sử dụng camera stereo để ước tính vị trí và tạo bản đồ điểm 3D
- **IMU Fusion:** Tích hợp dữ liệu từ MPU6050 để nâng cao độ chính xác góc quay
- **Extended Kalman Filter (EKF):** Kết hợp tối ưu dữ liệu từ camera và IMU
- **ORB-SLAM3 Support:** Hỗ trợ tích hợp ORB-SLAM3 cho độ chính xác cao hơn

**Đầu vào:**
- Cặp ảnh stereo từ 2 camera USB (640x480, 30fps)
- Dữ liệu gia tốc kế và con quay hồi chuyển từ IMU (100Hz)

**Đầu ra:**
- Pose 6-DoF: Vị trí (x, y, z) và hướng (roll, pitch, yaw)
- Map points: Điểm đặc trưng 3D của môi trường
- Tracking status: Trạng thái theo dõi (initialized, tracking, lost)

**Hiệu năng:**
- Tần suất xử lý: 10-30 FPS (tùy phần cứng)
- Độ chính xác vị trí: ±5cm (điều kiện tốt)
- Độ chính xác góc: ±2° (với IMU fusion)

#### 2.1.2. Lập bản đồ 2D (2D Occupancy Grid Mapping)
**Mô tả:** Tạo bản đồ lưới chiếm chỗ 2D để biểu diễn không gian được khám phá

**Thuật toán:**
- Grid-based mapping với ray tracing
- Bresenham line algorithm để đánh dấu vùng trống
- Frontier detection để phát hiện vùng chưa khám phá

**Thông số:**
- Kích thước bản đồ: 10m × 10m (có thể mở rộng)
- Độ phân giải: 5cm/cell (200×200 cells)
- Giá trị cell:
  - `-1`: Chưa khám phá
  - `0`: Không gian trống
  - `100`: Vật cản/tường

**Cập nhật:**
- Realtime update từ depth map của stereo camera
- Fusion với SLAM pose để đảm bảo tính nhất quán

#### 2.1.3. Autonomous Exploration (Khám phá tự động)
**Mô tả:** Robot tự động khám phá môi trường chưa biết

**Chiến lược:**
- **Frontier-based exploration:** Tìm và di chuyển đến biên giới giữa vùng đã biết và chưa biết
- **Nearest frontier selection:** Chọn frontier gần nhất để tối ưu hóa thời gian
- **Dynamic replanning:** Tính toán lại đường đi khi gặp vật cản mới

**Quy trình:**
1. Phát hiện frontiers trên bản đồ
2. Chọn frontier gần nhất và truy cập được
3. Lập kế hoạch đường đi đến frontier
4. Di chuyển và tránh vật cản
5. Lặp lại cho đến khi khám phá hết không gian

#### 2.1.4. Obstacle Detection & Avoidance (Phát hiện và tránh vật cản)
**Mô tả:** Hệ thống phát hiện vật cản thời gian thực từ disparity map

**Vùng phát hiện:**
- **Front Center:** 0°, khoảng cách an toàn 30cm
- **Front Left:** -45° đến 0°, khoảng cách an toàn 30cm
- **Front Right:** 0° đến 45°, khoảng cách an toàn 30cm
- **Left Side:** -90° đến -45°, khoảng cách an toàn 21cm
- **Right Side:** 45° đến 90°, khoảng cách an toàn 21cm

**Mức độ cảnh báo:**
- **Safe:** Không có vật cản trong phạm vi an toàn
- **Warning:** Vật cản trong khoảng 15-30cm, giảm tốc
- **Critical:** Vật cản < 15cm, dừng khẩn cấp

**Hành động:**
- **Stop:** Dừng ngay lập tức
- **Slow down:** Giảm tốc độ xuống 50%
- **Turn left/right:** Rẽ trái/phải để tránh
- **Reverse:** Lùi lại nếu bị kẹt

**Tính năng nâng cao:**
- Obstacle history (5 frames) để lọc nhiễu
- Hysteresis trong quyết định rẽ để tránh dao động
- Tích hợp vận tốc hiện tại để điều chỉnh khoảng cách an toàn

#### 2.1.5. Differential Drive Control (Điều khiển di chuyển)
**Mô tả:** Hệ thống điều khiển động cơ bước NEMA 17 qua driver A4988

**Kiểu điều khiển:**
- **Velocity control:** Điều khiển vận tốc tuyến tính và góc
- **Differential drive kinematics:** Tính toán vận tốc bánh trái/phải
- **Safety limits:** Giới hạn tốc độ và gia tốc tối đa

**Lệnh điều khiển:**
```python
set_velocity(linear, angular)
# linear: m/s (-0.3 đến 0.3)
# angular: rad/s (-1.5 đến 1.5)
```

**Tính năng an toàn:**
- Watchdog timer: Tự động dừng nếu không nhận lệnh trong 1s
- Emergency stop: Dừng khẩn cấp khi phát hiện lỗi
- Gradual acceleration/deceleration: Tăng/giảm tốc mượt mà

#### 2.1.6. Web Dashboard
**Mô tả:** Giao diện web để giám sát và điều khiển robot

**Tính năng:**
- **Live video:** Xem video trực tiếp từ camera stereo
- **Depth map visualization:** Hiển thị bản đồ độ sâu
- **Occupancy grid:** Xem bản đồ 2D được tạo
- **SLAM status:** Trạng thái SLAM, số điểm trên bản đồ
- **IMU data:** Góc quay, gia tốc
- **Manual control:** Điều khiển bằng bàn phím (WASD)
- **Autonomous control:** Bật/tắt chế độ tự động
- **Statistics:** Khoảng cách đi được, thời gian hoạt động

**Giao thức:**
- HTTP + WebSocket cho realtime data
- Flask backend
- HTML5/JavaScript/Canvas frontend

#### 2.1.7. Sensor Fusion (Kết hợp cảm biến)
**Mô tả:** Tích hợp nhiều nguồn cảm biến để tăng độ tin cậy

**Cảm biến:**
- **Stereo Camera:** Vị trí và bản đồ từ thị giác
- **MPU6050 IMU:** Góc quay và gia tốc
- **Motor Encoders:** Odometry từ encoder bánh xe (tương lai)

**Phương pháp fusion:**
- **Extended Kalman Filter (EKF):**
  - State: [x, y, θ, vx, vy, ω]
  - Prediction: Sử dụng IMU gyroscope
  - Update: Sử dụng Visual Odometry
- **Complementary Filter:** 
  - θ_fused = α × θ_visual + (1-α) × θ_imu
  - α = 0.7 (visual weight)

**Lợi ích:**
- Giảm drift của visual odometry
- Cải thiện độ chính xác góc quay
- Tăng tần suất cập nhật pose (100Hz thay vì 30Hz)

### 2.2. Tính năng kỹ thuật bổ sung

- **Camera Calibration:** Auto-load calibration từ file .npz
- **Manual Camera Settings:** Cố định exposure/gain để tránh flicker
- **Logging System:** Ghi log chi tiết với timestamp và level
- **Threading Architecture:** Đa luồng để xử lý song song camera, SLAM, motor
- **GPIO Safety:** Proper cleanup khi tắt chương trình

---

## 3. THÔNG SỐ KỸ THUẬT

### 3.1. Phần cứng

#### 3.1.1. Bộ xử lý trung tâm
- **Model:** NVIDIA Jetson Nano Developer Kit
- **CPU:** Quad-core ARM Cortex-A57 @ 1.43 GHz
- **GPU:** 128-core NVIDIA Maxwell
- **RAM:** 4GB 64-bit LPDDR4
- **Storage:** MicroSD (khuyến nghị 64GB+)
- **Nguồn:** 5V 4A (DC barrel jack)

#### 3.1.2. Hệ thống thị giác
- **Loại:** Dual USB Camera (Stereo Vision)
- **Model:** USB Camera (Logitech/Generic)
- **Độ phân giải:** 640×480 pixels
- **Frame rate:** 30 FPS
- **Baseline:** 140mm (khoảng cách giữa 2 camera)
- **Lens FOV:** ~60-70° (tùy loại camera)
- **Giao tiếp:** USB 2.0/3.0
- **Exposure:** Manual mode (fixed exposure -6)
- **Gain:** Manual gain 50

**Yêu cầu calibration:**
- Intrinsic parameters: K_left, D_left, K_right, D_right
- Extrinsic parameters: R (rotation), T (translation)
- File format: NumPy .npz
- Calibration tool: OpenCV stereo calibration

#### 3.1.3. Cảm biến quán tính (IMU)
- **Model:** MPU6050 6-axis IMU
- **Accelerometer:** ±2g, ±4g, ±8g, ±16g (configurable)
- **Gyroscope:** ±250°/s, ±500°/s, ±1000°/s, ±2000°/s (configurable)
- **Cấu hình sử dụng:** ±2g, ±250°/s
- **Giao tiếp:** I2C (bus 1 trên Jetson Nano)
- **I2C Address:** 0x68 (default)
- **Sample rate:** 100 Hz
- **Nhiệt độ hoạt động:** -40°C đến +85°C
- **Điện áp:** 3.3V (từ Jetson)

**Kết nối I2C:**
- SDA: Jetson Pin 3 (GPIO 2)
- SCL: Jetson Pin 5 (GPIO 3)
- VCC: 3.3V
- GND: GND

#### 3.1.4. Hệ thống truyền động

**Motor:**
- **Loại:** NEMA 17 Stepper Motor
- **Bước/vòng:** 200 steps/rev (1.8°/step)
- **Dòng định mức:** 1.5A (typical)
- **Điện áp:** 12V
- **Mô-men xoắn:** ~0.4 Nm (40 kg·cm)
- **Số lượng:** 2 motors (trái + phải)

**Driver:**
- **Model:** A4988 Stepper Motor Driver
- **Microstepping:** 1, 1/2, 1/4, 1/8, 1/16 step
- **Cấu hình sử dụng:** 1/16 step (3200 steps/rev)
- **Điện áp input:** 8-35V (sử dụng 12V)
- **Dòng tối đa:** 2A per coil
- **Logic voltage:** 3.3-5V (compatible với Jetson 3.3V GPIO)
- **VREF:** 0.42V (cho motor 1.5A)
  - VREF = I_max × 8 × R_sense
  - VREF = 1.5 × 8 × 0.05 = 0.6V (maximum)
  - VREF = 1.0 × 8 × 0.05 = 0.4V (recommended)

**Kết nối GPIO (Motor Trái):**
- STEP: Pin 33 (GPIO 13)
- DIR: Pin 35 (GPIO 19)
- ENABLE: Pin 37 (GPIO 26)
- MS1: Pin 29 (GPIO 5)
- MS2: Pin 31 (GPIO 6)
- MS3: Pin 11 (GPIO 17)

**Kết nối GPIO (Motor Phải):**
- STEP: Pin 32 (GPIO 12)
- DIR: Pin 36 (GPIO 16)
- ENABLE: Pin 38 (GPIO 20)
- MS1: Pin 13 (GPIO 27)
- MS2: Pin 15 (GPIO 22)
- MS3: Pin 16 (GPIO 23)

**Hệ thống bánh xe:**
- **Đường kính bánh xe:** 66mm
- **Khoảng cách giữa 2 bánh (wheelbase):** 275mm
- **Loại bánh:** Cao su chống trượt
- **Loại dẫn động:** Direct drive với hộp số

**Hộp số:**
- **Loại:** Belt drive với encoder teeth
- **Tỷ số truyền:** 38/18 = 2.111:1
  - Motor gear: 18 răng
  - Intermediate: 18 răng  
  - Wheel gear: 38 răng
- **Final ratio:** Motor quay 2.111 vòng = bánh xe quay 1 vòng

#### 3.1.5. Nguồn điện
- **Pin/Battery:** Li-Po 3S (11.1V nominal, 12.6V fully charged)
- **Dung lượng:** 5000mAh (khuyến nghị)
- **BMS:** Battery Management System với over-discharge protection
- **Buck converter 5V:** Để cấp nguồn cho Jetson Nano (5V 4A)
- **Nguồn 12V trực tiếp:** Cho motor drivers (A4988)
- **Thời gian hoạt động:** ~2-3 giờ (tùy tải)

**Sơ đồ nguồn:**
```
Battery 12V (Li-Po 3S)
├── Buck Converter (12V→5V 4A) → Jetson Nano
├── A4988 Driver 1 (VMOT) → Motor Trái
└── A4988 Driver 2 (VMOT) → Motor Phải
```

#### 3.1.6. Khung robot (Chassis)

**Vật liệu:**
- Acrylic hoặc nhôm tấm 3mm-5mm
- Standoffs kim loại M3 và M4
- Ốc vít M3, M4

**Kích thước:**
- Chiều dài: ~300mm
- Chiều rộng: ~250mm (bao gồm cả bánh xe)
- Chiều cao: ~150mm (không kể antenna/cảm biến)

**Cấu trúc:**
- Tầng 1 (dưới): Motors, drivers, battery
- Tầng 2 (giữa): Jetson Nano, camera mount
- Tầng 3 (trên - optional): Các cảm biến bổ sung

### 3.2. Phần mềm

#### 3.2.1. Hệ điều hành & Platform
- **OS:** Ubuntu 18.04 LTS (JetPack 4.6)
- **Kernel:** Linux 4.9
- **Python:** 3.6+
- **CUDA:** 10.2 (for ORB-SLAM3)

#### 3.2.2. Dependencies chính
```
opencv-python >= 4.5.0
numpy >= 1.19.0
smbus2 >= 0.4.0
Jetson.GPIO >= 2.0.17
Flask >= 2.0.0
Flask-SocketIO >= 5.0.0
scipy >= 1.5.0
```

#### 3.2.3. Cấu trúc project
```
Ver2/
├── main.py                      # Điểm khởi chạy chính
├── jetson_motor_controller.py  # Điều khiển motor A4988
├── stereo_camera.py             # Hệ thống camera stereo
├── mpu6050_imu.py               # Driver MPU6050
├── orb_slam3_wrapper.py         # Wrapper cho ORB-SLAM3
├── autonomous_mapper.py         # Autonomous exploration
├── obstacle_detector.py         # Phát hiện vật cản
├── imu_fusion.py                # Sensor fusion (EKF)
├── web_server.py                # Web dashboard
├── calibration.npz              # Camera calibration data
├── templates/
│   └── index.html               # Web UI
└── *.md                         # Documentation
```

#### 3.2.4. Configuration (trong main.py)

**Camera config:**
```python
'camera': {
    'left_id': 1,
    'right_id': 0,
    'width': 640,
    'height': 480,
    'fps': 30,
    'stereo_baseline': 0.14,  # 140mm
    'use_manual_settings': True,
    'manual_exposure': -6,
    'manual_gain': 50
}
```

**Motor config:**
```python
'motor': {
    'wheel_diameter': 0.066,      # 66mm
    'wheel_base': 0.275,          # 275mm
    'steps_per_rev': 200,
    'microsteps': 16,
    'gear_ratio': 2.111,          # 38/18
    'max_linear_speed': 0.3,      # m/s
    'max_angular_speed': 1.5      # rad/s
}
```

**IMU config:**
```python
'imu': {
    'bus_number': 1,
    'address': 0x68,
    'gyro_range': 250,            # deg/s
    'accel_range': 2,             # g
    'use_ekf': True,
    'fusion_weight': 0.3
}
```

**Mapping config:**
```python
'mapping': {
    'grid_width': 10.0,           # meters
    'grid_height': 10.0,
    'resolution': 0.05            # 5cm per cell
}
```

**Obstacle detection config:**
```python
'obstacle_detection': {
    'min_safe_distance': 0.3,     # 30cm
    'critical_distance': 0.15,    # 15cm
    'detection_width': 0.5,       # 50cm
    'max_range': 2.0              # 2m
}
```

---

## 4. PHƯƠNG PHÁP TÍNH TOÁN CHI TIẾT

### 4.1. Kinematics - Differential Drive

#### 4.1.1. Forward Kinematics
Cho vận tốc bánh trái $v_L$ và bánh phải $v_R$, tính vận tốc tuyến tính $v$ và góc $\omega$:

$$v = \frac{v_L + v_R}{2}$$

$$\omega = \frac{v_R - v_L}{L}$$

Trong đó:
- $v$: Vận tốc tuyến tính (m/s)
- $\omega$: Vận tốc góc (rad/s)
- $L$: Khoảng cách giữa 2 bánh (wheelbase) = 0.275m

#### 4.1.2. Inverse Kinematics
Cho vận tốc mong muốn $v$ và $\omega$, tính vận tốc từng bánh:

$$v_L = v - \frac{\omega \cdot L}{2}$$

$$v_R = v + \frac{\omega \cdot L}{2}$$

#### 4.1.3. Chuyển đổi từ vận tốc bánh sang steps/s

**Bước 1: Tính chu vi bánh xe**
$$C = \pi \cdot d = 3.14159 \times 0.066 = 0.2073 \text{ m}$$

**Bước 2: Tính số steps cho 1 vòng bánh**
$$\text{steps\_per\_wheel\_rev} = \text{steps\_per\_motor\_rev} \times \text{gear\_ratio}$$
$$= (200 \times 16) \times 2.111 = 6755.2 \text{ steps}$$

Trong đó:
- 200: Số bước/vòng của motor
- 16: Microstepping (1/16 step)
- 2.111: Tỷ số truyền

**Bước 3: Tính steps per meter**
$$\text{steps\_per\_meter} = \frac{\text{steps\_per\_wheel\_rev}}{C} = \frac{6755.2}{0.2073} = 32589 \text{ steps/m}$$

**Bước 4: Chuyển vận tốc (m/s) sang steps/s**
$$\text{steps\_per\_sec} = v \times \text{steps\_per\_meter}$$

**Ví dụ:**
- $v_L = 0.1$ m/s
- steps/s = $0.1 \times 32589 = 3258.9$ steps/s

**Code implementation:**
```python
def velocity_to_steps(self, velocity_ms):
    """Convert m/s to steps/s"""
    return velocity_ms * self.steps_per_meter

def set_velocity(self, linear, angular):
    """Set robot velocity (m/s, rad/s)"""
    # Inverse kinematics
    v_left = linear - (angular * self.wheel_base / 2.0)
    v_right = linear + (angular * self.wheel_base / 2.0)
    
    # Convert to steps/s
    steps_left = self.velocity_to_steps(v_left)
    steps_right = self.velocity_to_steps(v_right)
    
    # Command motors
    self.left_motor.set_speed(steps_left)
    self.right_motor.set_speed(steps_right)
```

### 4.2. Stereo Vision - Depth Calculation

#### 4.2.1. Công thức tính độ sâu từ disparity

$$Z = \frac{f \cdot B}{d}$$

Trong đó:
- $Z$: Độ sâu (khoảng cách đến vật) (meters)
- $f$: Tiêu cự camera (pixels) - từ ma trận K
- $B$: Baseline (khoảng cách giữa 2 camera) = 0.14m
- $d$: Disparity (độ chênh lệch pixel giữa ảnh trái và phải)

**Ví dụ:**
- $f = 500$ pixels (từ K[0,0])
- $B = 0.14$ m
- $d = 50$ pixels
- $Z = \frac{500 \times 0.14}{50} = 1.4$ m

#### 4.2.2. Chuyển từ ảnh 2D sang tọa độ 3D

Cho điểm $(u, v)$ trên ảnh và độ sâu $Z$:

$$X = \frac{(u - c_x) \cdot Z}{f_x}$$

$$Y = \frac{(v - c_y) \cdot Z}{f_y}$$

$$Z = Z$$

Trong đó $K$ là ma trận intrinsic:
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

**Code implementation:**
```python
def disparity_to_depth(disparity, K, baseline):
    """Convert disparity map to depth map"""
    f = K[0, 0]  # focal length
    
    # Avoid division by zero
    disparity[disparity < 1] = 1
    
    # Z = f * B / d
    depth = (f * baseline) / disparity
    return depth

def pixel_to_3d(u, v, depth, K):
    """Convert 2D pixel + depth to 3D point"""
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
    
    return X, Y, Z
```

#### 4.2.3. Stereo Matching
Algorithm sử dụng: **Semi-Global Block Matching (SGBM)**

**Parameters:**
```python
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,      # Must be divisible by 16
    blockSize=5,             # Odd number, typically 3-11
    P1=8 * 3 * 5**2,         # Penalty for small disparity changes
    P2=32 * 3 * 5**2,        # Penalty for large disparity changes
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)
```

### 4.3. Visual SLAM - Feature Tracking

#### 4.3.1. Feature Detection (ORB Features)

**ORB = Oriented FAST and Rotated BRIEF**

**Số lượng features:**
- 1000-2000 features per frame (configurable)
- Distributed uniformly across image

**Feature matching:**
- Brute-Force matcher với Hamming distance
- Lowe's ratio test: ratio = 0.7

#### 4.3.2. Essential Matrix & Pose Recovery

**Essential Matrix từ feature matches:**
$$E = [t]_\times R$$

Trong đó:
- $R$: Rotation matrix (3×3)
- $t$: Translation vector (3×1)
- $[t]_\times$ là skew-symmetric matrix của $t$

**Recover pose:**
```python
E, mask = cv2.findEssentialMat(
    points1, points2, K, 
    method=cv2.RANSAC, 
    prob=0.999, 
    threshold=1.0
)

_, R, t, mask = cv2.recoverPose(E, points1, points2, K)
```

**Tích lũy pose:**
$$T_{new} = T_{prev} \cdot \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

#### 4.3.3. Triangulation - Tính 3D từ 2 view

Cho 2 camera với pose $P_1$ và $P_2$, và điểm tương ứng $(u_1, v_1)$, $(u_2, v_2)$:

$$P_1 = K [I | 0]$$
$$P_2 = K [R | t]$$

Sử dụng DLT (Direct Linear Transform):
```python
points_4d = cv2.triangulatePoints(P1, P2, points1, points2)
points_3d = points_4d[:3] / points_4d[3]  # Normalize
```

### 4.4. IMU Sensor Fusion - Extended Kalman Filter

#### 4.4.1. State Vector
$$\mathbf{x} = [x, y, \theta, v_x, v_y, \omega]^T$$

Trong đó:
- $(x, y, \theta)$: Vị trí và góc quay
- $(v_x, v_y, \omega)$: Vận tốc tuyến tính và góc

#### 4.4.2. Prediction Step

**State prediction:**
$$\hat{x}_{k+1} = f(\hat{x}_k, u_k)$$

$$\begin{bmatrix} x \\ y \\ \theta \\ v_x \\ v_y \\ \omega \end{bmatrix}_{k+1} = \begin{bmatrix} x + (v_x \cos\theta - v_y \sin\theta) \Delta t \\ y + (v_x \sin\theta + v_y \cos\theta) \Delta t \\ \theta + \omega \Delta t \\ v_x \\ v_y \\ \omega_{imu} \end{bmatrix}_k$$

**Jacobian:**
$$F = \frac{\partial f}{\partial x} = \begin{bmatrix} 1 & 0 & -(v_x \sin\theta + v_y \cos\theta)\Delta t & \cos\theta \Delta t & -\sin\theta \Delta t & 0 \\ 0 & 1 & (v_x \cos\theta - v_y \sin\theta)\Delta t & \sin\theta \Delta t & \cos\theta \Delta t & 0 \\ 0 & 0 & 1 & 0 & 0 & \Delta t \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix}$$

**Covariance prediction:**
$$P_{k+1} = F P_k F^T + Q$$

#### 4.4.3. Update Step (Visual Odometry)

**Measurement model:**
$$\mathbf{z} = H \mathbf{x} + v$$

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 & 0 & 0 \end{bmatrix}$$

**Innovation:**
$$\mathbf{y} = \mathbf{z} - H\hat{\mathbf{x}}$$

**Innovation covariance:**
$$S = H P H^T + R$$

**Kalman Gain:**
$$K = P H^T S^{-1}$$

**State update:**
$$\hat{\mathbf{x}}_{new} = \hat{\mathbf{x}} + K \mathbf{y}$$

**Covariance update:**
$$P_{new} = (I - KH) P$$

**Code implementation:**
```python
def predict(self, dt, gyro_z):
    """EKF prediction step"""
    x, y, theta, vx, vy, omega = self.state
    
    # State prediction
    new_x = x + (vx * np.cos(theta) - vy * np.sin(theta)) * dt
    new_y = y + (vx * np.sin(theta) + vy * np.cos(theta)) * dt
    new_theta = theta + omega * dt
    new_omega = gyro_z  # From IMU
    
    self.state = np.array([new_x, new_y, new_theta, vx, vy, new_omega])
    
    # Jacobian F
    F = np.eye(6)
    F[0, 2] = -(vx * np.sin(theta) + vy * np.cos(theta)) * dt
    F[0, 3] = np.cos(theta) * dt
    F[0, 4] = -np.sin(theta) * dt
    F[1, 2] = (vx * np.cos(theta) - vy * np.sin(theta)) * dt
    F[1, 3] = np.sin(theta) * dt
    F[1, 4] = np.cos(theta) * dt
    F[2, 5] = dt
    
    # Covariance prediction
    self.P = F @ self.P @ F.T + self.Q

def update_visual(self, measurement):
    """EKF update from visual odometry"""
    H = np.zeros((3, 6))
    H[0, 0] = 1  # x
    H[1, 1] = 1  # y
    H[2, 2] = 1  # theta
    
    z = np.array(measurement)
    y = z - self.state[:3]
    
    # Normalize angle
    y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
    
    S = H @ self.P @ H.T + self.R_visual
    K = self.P @ H.T @ np.linalg.inv(S)
    
    self.state = self.state + K @ y
    self.P = (np.eye(6) - K @ H) @ self.P
```

### 4.5. Obstacle Detection from Depth Map

#### 4.5.1. Tạo điểm mây 3D (Point Cloud)

Cho mỗi pixel $(u, v)$ có depth $Z$:

$$X = \frac{(u - c_x) \cdot Z}{f_x}$$
$$Y = \frac{(v - c_y) \cdot Z}{f_y}$$
$$Z = Z$$

Filter điểm trong vùng ROI (Region of Interest):
- Width: ±0.25m (robot width + margin)
- Height: 0 to 0.4m (sensor height)
- Depth: 0.1m to 2.0m (detection range)

#### 4.5.2. Phân loại vùng (Zone Classification)

Chia không gian thành 5 zones:
```python
zones = {
    'front_center': {'angle_range': (-15, 15), 'distance': 0.3},
    'front_left': {'angle_range': (-45, -15), 'distance': 0.3},
    'front_right': {'angle_range': (15, 45), 'distance': 0.3},
    'left': {'angle_range': (-90, -45), 'distance': 0.21},
    'right': {'angle_range': (45, 90), 'distance': 0.21}
}
```

Cho điểm 3D $(X, Y, Z)$:
$$\text{distance} = \sqrt{X^2 + Z^2}$$
$$\text{angle} = \arctan2(X, Z) \times \frac{180}{\pi}$$

#### 4.5.3. Quyết định hành động

**Algorithm:**
```python
def determine_action(zone_status, velocity):
    if zone_status['front_center']['critical']:
        return {'action': 'EMERGENCY_STOP'}
    
    if zone_status['front_center']['has_obstacle']:
        if zone_status['front_left']['clear'] and zone_status['front_right']['has_obstacle']:
            return {'action': 'TURN_LEFT', 'angular': 0.5}
        elif zone_status['front_right']['clear']:
            return {'action': 'TURN_RIGHT', 'angular': -0.5}
        else:
            return {'action': 'REVERSE', 'linear': -0.1}
    
    return {'action': 'CONTINUE'}
```

**Hysteresis cho quyết định:**
- Lưu lịch sử 5 frames
- Chỉ thay đổi quyết định nếu > 60% frames đồng ý

### 4.6. Occupancy Grid Mapping

#### 4.6.1. Ray Tracing - Bresenham Algorithm

Cho robot tại $(x_0, y_0)$ và phát hiện vật cản tại $(x_1, y_1)$:

**Chuyển sang grid coordinates:**
$$g_x = \lfloor \frac{x + \text{origin}_x}{\text{resolution}} \rfloor$$
$$g_y = \lfloor \frac{y + \text{origin}_y}{\text{resolution}} \rfloor$$

**Bresenham line:**
```python
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    
    return points
```

**Cập nhật grid:**
- Tất cả điểm trên ray (trừ endpoint): `grid[y, x] = 0` (free)
- Endpoint: `grid[y, x] = 100` (occupied)

#### 4.6.2. Frontier Detection

**Định nghĩa Frontier:**
Cell là frontier nếu:
1. Cell đó là free (`grid[y, x] == 0`)
2. Ít nhất 1 neighbor là unknown (`grid[y±1, x±1] == -1`)

**Algorithm:**
```python
def get_frontiers():
    frontiers = []
    for y in range(1, height-1):
        for x in range(1, width-1):
            if grid[y, x] == 0:  # Free cell
                neighbors = [
                    grid[y-1, x], grid[y+1, x],
                    grid[y, x-1], grid[y, x+1]
                ]
                if -1 in neighbors:  # Has unknown neighbor
                    frontiers.append((x, y))
    return frontiers
```

#### 4.6.3. Nearest Frontier Selection

**Distance metric:** Euclidean distance
$$d = \sqrt{(x_{robot} - x_{frontier})^2 + (y_{robot} - y_{frontier})^2}$$

**Selection:**
```python
def find_nearest_frontier(robot_x, robot_y):
    frontiers = get_frontiers()
    
    robot_gx, robot_gy = world_to_grid(robot_x, robot_y)
    
    min_dist = float('inf')
    nearest = None
    
    for fx, fy in frontiers:
        dist = sqrt((fx - robot_gx)**2 + (fy - robot_gy)**2)
        if dist < min_dist:
            min_dist = dist
            nearest = (fx, fy)
    
    return grid_to_world(nearest[0], nearest[1])
```

---

## 5. CẤU HÌNH CHI TIẾT

### 5.1. Cấu hình phần cứng

#### 5.1.1. Sơ đồ kết nối tổng thể

```
                    ROBOT SYSTEM ARCHITECTURE
                    
┌─────────────────────────────────────────────────────────────┐
│                      JETSON NANO                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │  GPIO    │  │   USB    │  │   I2C    │  │ Ethernet │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
└───────┼─────────────┼─────────────┼─────────────┼──────────┘
        │             │             │             │
        │             │             │             │
    ┌───▼───┐     ┌───▼───┐     ┌───▼───┐   ┌────▼────┐
    │ A4988 │     │Camera │     │MPU6050│   │ Web     │
    │Driver │     │Stereo │     │  IMU  │   │ Client  │
    └───┬───┘     └───────┘     └───────┘   └─────────┘
        │
    ┌───▼───┐
    │ NEMA  │
    │  17   │
    │ Motor │
    └───────┘
```

#### 5.1.2. Pin mapping chi tiết

**Jetson Nano J41 GPIO Header:**
```
Pin 1  [3.3V]       Pin 2  [5V]
Pin 3  [I2C_SDA]    Pin 4  [5V]
Pin 5  [I2C_SCL]    Pin 6  [GND]
Pin 7  [GPIO 4]     Pin 8  [UART_TX]
...
Pin 11 [GPIO 17]    Pin 12 [GPIO 18]    # Left MS3, Right STEP
Pin 13 [GPIO 27]    Pin 14 [GND]        # Right MS1
Pin 15 [GPIO 22]    Pin 16 [GPIO 23]    # Right MS2, MS3
...
Pin 29 [GPIO 5]     Pin 30 [GND]        # Left MS1
Pin 31 [GPIO 6]     Pin 32 [GPIO 12]    # Left MS2, Right STEP
Pin 33 [GPIO 13]    Pin 34 [GND]        # Left STEP
Pin 35 [GPIO 19]    Pin 36 [GPIO 16]    # Left DIR, Right DIR
Pin 37 [GPIO 26]    Pin 38 [GPIO 20]    # Left EN, Right EN
Pin 39 [GND]        Pin 40 [GPIO 21]
```

**Complete GPIO Mapping:**
| Component | Function | Pin | GPIO | Wire Color (Suggestion) |
|-----------|---------|-----|------|------------------------|
| Left Motor | STEP | 33 | 13 | Yellow |
| Left Motor | DIR | 35 | 19 | Green |
| Left Motor | ENABLE | 37 | 26 | Blue |
| Left Motor | MS1 | 29 | 5 | Orange |
| Left Motor | MS2 | 31 | 6 | Red |
| Left Motor | MS3 | 11 | 17 | Brown |
| Right Motor | STEP | 32 | 12 | Yellow |
| Right Motor | DIR | 36 | 16 | Green |
| Right Motor | ENABLE | 38 | 20 | Blue |
| Right Motor | MS1 | 13 | 27 | Orange |
| Right Motor | MS2 | 15 | 22 | Red |
| Right Motor | MS3 | 16 | 23 | Brown |
| MPU6050 | SDA | 3 | 2 | White |
| MPU6050 | SCL | 5 | 3 | Gray |
| Common | GND | 6,14,20,25,30,34,39 | - | Black |
| Common | 3.3V | 1,17 | - | Red |

#### 5.1.3. A4988 Driver Wiring

**Single A4988 Driver:**
```
         A4988 DRIVER
    ┌─────────────────┐
VMOT●                 ● 
GND ●                 ●2B   ─┐
    │                 │      │ Motor Coil B
2A  ●                 ●2A    │
    │                 │     ─┘
1A  ●     A4988       ●1B  ─┐
    │                 │      │ Motor Coil A
1B  ●                 ●1A    │
    │                 │     ─┘
VDD ●                 ●GND
    │                 │
DIR ●    [VREF POT]   ●STEP
    │        │        │
SLEEP●       ↓        ●RESET
    │                 │
MS3 ●                 ●MS2
    │                 │
MS1 ●                 ●ENABLE
    └─────────────────┘
```

**Power connections:**
- VMOT: 12V from battery
- GND (power): Common ground with battery and Jetson
- VDD: Usually self-powered from onboard regulator (3-5V)
- GND (logic): Common ground

**Signal connections:**
- STEP, DIR, ENABLE: From Jetson GPIO
- MS1, MS2, MS3: From Jetson GPIO hoặc tied to VDD/GND
- RESET, SLEEP: Tied to VDD (always active)

**Motor connections:**
- 1A, 1B: Coil A của NEMA 17
- 2A, 2B: Coil B của NEMA 17

#### 5.1.4. VREF Adjustment

**Công thức:**
$$V_{REF} = I_{max} \times 8 \times R_{sense}$$

Với A4988 có $R_{sense} = 0.05\Omega$:
$$V_{REF} = I_{max} \times 8 \times 0.05 = I_{max} \times 0.4$$

**Cho motor 1.5A:**
$$V_{REF} = 1.5 \times 0.4 = 0.6V \text{ (max)}$$

**Recommended (70%):**
$$V_{REF} = 1.0 \times 0.4 = 0.4V$$

**Đo VREF:**
1. Bật nguồn 12V cho A4988
2. Multimeter ở chế độ DC Voltage
3. Đo giữa potentiometer wiper và GND
4. Vặn potentiometer bằng tuốc nơ vít nhỏ để điều chỉnh

#### 5.1.5. Stereo Camera Mount

**Yêu cầu:**
- Khoảng cách chính xác: 140mm ± 1mm
- Song song hoàn toàn (tránh lens không cùng phương)
- Gắn chắc chắn (tránh rung)
- Height: ~20-30cm from ground

**Vật liệu:**
- Acrylic bar hoặc aluminum profile
- Camera mount brackets (in 3D hoặc mua sẵn)
- M3 screws

**Kiểm tra alignment:**
- Chụp ảnh cùng lúc 2 camera
- Kiểm tra epipolar lines (phải nằm ngang)
- Nếu lệch → calibration sẽ kém

### 5.2. Cấu hình cơ khí

#### 5.2.1. Hệ thống truyền động

**Belt Drive System:**
```
    Motor (NEMA 17)
           │
           │ Pulley 18T
           ▼
    ┌──────────┐
    │   Belt   │
    └──────────┘
           │
           │ Pulley 38T
           ▼
       Wheel (66mm)
```

**Specifications:**
- Motor pulley: 18 teeth (GT2 timing belt)
- Wheel pulley: 38 teeth (GT2 timing belt)
- Belt type: GT2 timing belt, 6mm width
- Belt length: Tính theo khoảng cách motor-wheel

**Tính chiều dài belt:**
$$L = 2C + \frac{\pi}{2}(D_1 + D_2) + \frac{(D_2 - D_1)^2}{4C}$$

Trong đó:
- $C$: Khoảng cách tâm giữa 2 pulley
- $D_1, D_2$: Đường kính pulley

**Tensioning:**
- Sử dụng slotted holes để điều chỉnh căng belt
- Belt tension: Đủ căng để không bị trượt, không quá căng gây mài mòn

#### 5.2.2. Castor Wheel (Bánh xe phụ)

**Vị trí:**
- 1-2 castor wheels ở phía trước/sau
- Để cân bằng robot (3-4 điểm tiếp đất)

**Loại:**
- Ball caster hoặc swivel caster
- Đường kính: 20-30mm
- Free-rolling (không ma sát cao)

#### 5.2.3. Center of Mass

**Yêu cầu:**
- Center of mass gần trục bánh xe chủ động
- Tránh nghiêng quá nhiều về phía trước/sau

**Distribution:**
- Battery (nặng) → Tầng dưới, giữa 2 bánh
- Jetson + Camera → Tầng giữa
- Sensors → Tầng trên (nhẹ)

#### 5.2.4. Dimensions Summary

| Thông số | Giá trị | Ghi chú |
|----------|---------|---------|
| Chiều dài | 300mm | Không kể sensors |
| Chiều rộng | 250mm | Bao gồm bánh xe |
| Chiều cao | 150mm | Không kể camera |
| Wheelbase | 275mm | Khoảng cách 2 bánh chủ |
| Wheel diameter | 66mm | Cao su, có rãnh |
| Ground clearance | 10-15mm | Tùy castor wheel |
| Tổng trọng lượng | ~2-2.5kg | Bao gồm battery |

### 5.3. Cấu hình phần mềm

#### 5.3.1. Cài đặt JetPack & CUDA

```bash
# Flash JetPack 4.6 (Ubuntu 18.04 + CUDA 10.2)
# Sử dụng NVIDIA SDK Manager từ host PC

# Sau khi flash, verify:
jetson_release
# Output: JetPack 4.6, L4T 32.6.1

# Check CUDA
nvcc --version
# Output: Cuda compilation tools, release 10.2
```

#### 5.3.2. Cài đặt Python dependencies

```bash
# Update system
sudo apt update
sudo apt upgrade

# Install Python 3 và pip
sudo apt install python3-pip python3-dev

# Install OpenCV với CUDA support (từ source - nâng cao)
# Hoặc dùng pre-built:
sudo apt install python3-opencv

# Install dependencies
pip3 install numpy scipy
pip3 install smbus2
pip3 install flask flask-socketio
pip3 install Jetson.GPIO

# For ORB-SLAM3 (optional, advanced):
sudo apt install libeigen3-dev libopencv-dev
# Build ORB-SLAM3 theo hướng dẫn official
```

#### 5.3.3. Camera Calibration

**Bước 1: Chuẩn bị checkerboard**
- In checkerboard pattern (9×6 hoặc 10×7 squares)
- Gắn lên bề mặt phẳng cứng (acrylic, gỗ)

**Bước 2: Chụp ảnh calibration**
```bash
# Chạy script capture (tự viết hoặc dùng OpenCV sample)
python3 calibrate_stereo.py

# Hướng dẫn:
# - Chụp 20-30 cặp ảnh stereo
# - Giữ checkerboard ở nhiều góc độ khác nhau
# - Đảm bảo checkerboard được detect rõ ràng
```

**Bước 3: Chạy calibration**
```python
import cv2
import numpy as np

# Stereo calibration
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    None, None, None, None, img_size,
    criteria=criteria,
    flags=cv2.CALIB_FIX_INTRINSIC
)

# Save to file
np.savez('calibration.npz',
    K_left=K_left, D_left=D_left,
    K_right=K_right, D_right=D_right,
    R=R, T=T, E=E, F=F
)
```

**Bước 4: Verify**
```bash
python3 check_calibration.py

# Xem stereo rectification
# Kiểm tra epipolar lines (phải ngang)
```

#### 5.3.4. IMU Calibration

**Enable I2C:**
```bash
# Check I2C bus
sudo i2cdetect -y -r 1

# Output should show 0x68 (MPU6050 address)
```

**Calibration procedure:**
```python
# Run IMU calibration
sudo python3 test_imu.py

# Hướng dẫn:
# 1. Đặt robot trên mặt phẳng ngang
# 2. KHÔNG di chuyển trong quá trình calibration
# 3. Script sẽ thu 500 samples và tính offset
# 4. Offset được lưu trong code (hoặc file)
```

**Verify calibration:**
```python
# Gyro drift < 0.5 deg/s
# Accel Z ~ 9.8 m/s² (1g)
# Accel X, Y ~ 0
```

#### 5.3.5. Auto-start on Boot

**Tạo systemd service:**
```bash
sudo nano /etc/systemd/system/robot.service
```

```ini
[Unit]
Description=Autonomous Mapping Robot
After=network.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/Robot_Slam/Ver2
ExecStart=/usr/bin/python3 main.py
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service

# Check status
sudo systemctl status robot.service

# View logs
journalctl -u robot.service -f
```

#### 5.3.6. Network Configuration

**WiFi setup:**
```bash
# Edit netplan
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"
```

```bash
sudo netplan apply
```

**Static IP (optional):**
```yaml
wifis:
  wlan0:
    dhcp4: no
    addresses: [192.168.1.100/24]
    gateway4: 192.168.1.1
    nameservers:
      addresses: [8.8.8.8, 8.8.4.4]
    access-points:
      "YOUR_SSID":
        password: "YOUR_PASSWORD"
```

**Find Jetson IP:**
```bash
hostname -I
# Hoặc
ip addr show wlan0
```

**Access web dashboard:**
```
http://<jetson-ip>:5000
```

---

## 6. ĐIỂM YẾU, TỒN TẠI VÀ CẢI TIẾN TƯƠNG LAI

### 6.1. Điểm yếu hiện tại

#### 6.1.1. Phần cứng

**Camera stereo USB:**
- **Vấn đề:** 
  - Baseline cố định 14cm → Giới hạn depth range
  - Không đồng bộ hardware → Có thể bị lệch frame
  - Phụ thuộc vào ánh sáng môi trường
  - Auto-exposure/white-balance gây mismatch màu sắc
  
- **Hậu quả:**
  - Depth accuracy giảm ở khoảng cách xa (>3m)
  - Feature matching kém khi ánh sáng thay đổi
  - Màu sắc không khớp giữa 2 camera → SLAM kém

**Motor stepper không có encoder:**
- **Vấn đề:**
  - Open-loop control → Không feedback
  - Skip steps khi tải lớn hoặc tốc độ cao
  - Odometry không chính xác
  
- **Hậu quả:**
  - Lỗi tích lũy theo thời gian
  - Không phát hiện được trượt bánh

**Battery life:**
- **Vấn đề:**
  - Jetson Nano tiêu thụ 10-15W
  - Motors tiêu thụ 15-30W (khi chạy)
  - 5000mAh chỉ đủ 2-3h
  
- **Hạn chế:**
  - Thời gian hoạt động liên tục ngắn
  - Cần charging station

#### 6.1.2. Thuật toán

**Simple Visual SLAM:**
- **Vấn đề:**
  - Không có loop closure → Drift tích lũy
  - Không có global optimization
  - Feature-based → Yêu cầu môi trường có texture
  
- **Hậu quả:**
  - Lỗi pose tăng dần theo thời gian
  - Không hoạt động tốt trong môi trường trơn (tường trắng)
  - Map không consistent khi quay về vị trí cũ

**Frontier-based exploration:**
- **Vấn đề:**
  - Chỉ chọn nearest frontier → Không tối ưu global
  - Không xét đến chi phí di chuyển (quay đầu, vật cản)
  - Có thể bỏ sót vài vùng nhỏ
  
- **Hậu quả:**
  - Đường đi không tối ưu
  - Thời gian khám phá lâu hơn cần thiết

**Obstacle detection:**
- **Vấn đề:**
  - Chỉ dựa vào depth map → Nhiễu cao
  - Không phát hiện vật cản trong suốt (kính)
  - Không phát hiện vật cản thấp hơn camera
  
- **Hạn chế:**
  - False positive cao trong điều kiện ánh sáng kém
  - Không an toàn 100%

#### 6.1.3. Hiệu năng

**Processing speed:**
- **Vấn đề:**
  - Visual SLAM chạy 10-15 FPS trên Jetson Nano
  - ORB-SLAM3 (nếu dùng) chỉ 5-8 FPS
  - Latency giữa sensing và action ~100-200ms
  
- **Hậu quả:**
  - Không thể di chuyển quá nhanh
  - Phản ứng chậm với vật cản đột ngột

**Memory:**
- **Vấn đề:**
  - Jetson Nano chỉ 4GB RAM
  - SLAM map lớn → Tốn RAM
  - Python không tối ưu bộ nhớ
  
- **Giới hạn:**
  - Không thể map diện tích quá rộng (>100m²)
  - Cần periodic map pruning

### 6.2. Tồn tại cần khắc phục

#### 6.2.1. Tồn tại nghiêm trọng

1. **Không có safety stop hardware**
   - Thiếu emergency stop button vật lý
   - Nên có: Nút stop cứng hoặc remote control override

2. **Không có battery monitoring**
   - Không đo voltage/current real-time
   - Cần: ADC để đo pin, auto park khi low battery

3. **Camera không waterproof**
   - Không chịu được bụi/nước
   - Cần: Housing bảo vệ

4. **Motor driver chưa có protection**
   - Không có current sensing feedback
   - Không có thermal protection
   - Nên upgrade: Driver có feedback hoặc add sensor

#### 6.2.2. Tồn tại vừa phải

1. **Web UI chưa responsive**
   - Giao diện web không tối ưu cho mobile
   - Cần: Responsive design CSS

2. **Logging chưa persistent**
   - Log chỉ lưu trong session
   - Cần: Database hoặc log analysis tool

3. **Không có path planning advanced**
   - Chưa có A* hoặc RRT
   - Chỉ go-to-goal trực tiếp
   - Cần: Implement A* trên occupancy grid

4. **IMU drift over time**
   - Complementary filter vẫn bị drift dài hạn
   - Cần: Magnetometer để absolute heading

### 6.3. Cải tiến tương lai

#### 6.3.1. Nâng cấp phần cứng (Ưu tiên cao)

**1. Thay thế sang DC motor + encoder**
- **Động cơ:** DC motor với encoder quang học (quadrature)
- **Driver:** Motor driver H-bridge (L298N, TB6612, hoặc chuyên nghiệp hơn: Sabertooth)
- **Lợi ích:**
  - Closed-loop control → Chính xác hơn
  - Feedback tốc độ thực tế
  - Phát hiện skip, trượt
  - Odometry chính xác
  
**2. Upgrade sang IMX219 Stereo Camera**
- **Camera:** Raspberry Pi Camera V2 (IMX219) do hai
- **Lợi ích:**
  - Hardware trigger → Đồng bộ perfect
  - CSI interface → Latency thấp hơn USB
  - Exposure/gain control tốt hơn
  - Hỗ trợ CUDA ISP pipeline

**3. Thêm LIDAR 2D**
- **Model:** RPLIDAR A1 hoặc A2
- **Lợi ích:**
  - 360° coverage
  - Chính xác hơn depth camera
  - Không phụ thuộc ánh sáng
  - Tích hợp SLAM 2D + Visual SLAM → Hybrid
  
**4. Thêm Magnetometer (Compass)**
- **Model:** HMC5883L hoặc QMC5883L
- **Lợi ích:**
  - Absolute heading (không drift)
  - Fusion với IMU gyro
  - Cải thiện orientation tracking

**5. Battery monitoring**
- **Hardware:** Voltage divider + ADC (ADS1115)
- **Software:** Đọc voltage, tính % pin
- **Feature:**
  - Warning khi pin thấp
  - Auto return to charging station
  - Graceful shutdown

#### 6.3.2. Nâng cấp thuật toán (Ưu tiên cao)

**1. Chuyển sang ORB-SLAM3 đầy đủ**
- **Hiện tại:** Simple Visual SLAM
- **Nâng cấp:** ORB-SLAM3 với loop closure
- **Lợi ích:**
  - Loop closure tự động → Sửa drift
  - Global bundle adjustment
  - Relocalization khi tracking lost
  - Map reuse (lưu map, load lại)

**2. Implement A* Path Planning**
- **Hiện tại:** Go-to-goal trực tiếp
- **Nâng cấp:** A* hoặc D* Lite
- **Lợi ích:**
  - Đường đi tối ưu tránh vật cản
  - Smooth trajectory
  - Dynamic replanning

**3. SLAM Backend với g2o**
- **Thêm:** Graph optimization
- **Lợi ích:**
  - Tối ưu hóa toàn bộ pose graph
  - Giảm drift
  - Consistent map

**4. Machine Learning cho Obstacle Detection**
- **Model:** YOLOv5-nano hoặc MobileNet-SSD
- **Train:** Phát hiện object (người, ghế, bàn, tường)
- **Lợi ích:**
  - Semantic understanding
  - Phát hiện động object (người đi)
  - Rule-based behavior (không đâm người)

**5. Multi-sensor Fusion nâng cao**
- **Sensor:** Camera + IMU + LIDAR + Encoder
- **Framework:** Robot Localization (ROS package) hoặc custom EKF
- **Lợi ích:**
  - Tăng độ tin cậy
  - Robust khi 1 sensor fail

#### 6.3.3. Tích hợp ROS (Robot Operating System)

**Chuyển toàn bộ hệ thống sang ROS/ROS2:**

**Lợi ích:**
- Standardized framework
- Reuse packages (gmapping, amcl, move_base)
- Simulation trong Gazebo
- Visualization với RViz
- Multi-robot coordination (tương lai)

**Key ROS packages:**
```
ros-melodic-slam-gmapping      # SLAM 2D
ros-melodic-navigation         # Navigation stack
ros-melodic-robot-localization # Sensor fusion
ros-melodic-stereo-image-proc  # Stereo processing
ros-melodic-rtabmap-ros        # Visual SLAM
```

**Architecture:**
```
Nodes:
- /camera_node          → Publish stereo images
- /imu_node             → Publish IMU data
- /motor_controller     → Subscribe cmd_vel, publish odometry
- /slam_node            → Subscribe images, IMU → Publish pose, map
- /obstacle_detector    → Subscribe depth → Publish obstacles
- /path_planner         → Subscribe map, goal → Publish path
- /navigation           → Subscribe obstacles, path → Publish cmd_vel
```

#### 6.3.4. Tính năng bổ sung

**1. Autonomous Charging**
- **Hardware:** Charging dock với contact pads
- **Software:** Detect dock, align, park
- **Benefit:** Unlimited operation time

**2. Cloud Integration**
- **Upload:** Map, logs, statistics
- **Download:** Firmware update, config
- **Dashboard:** Web monitoring từ xa

**3. Multi-floor Mapping**
- **Cảm biến:** Barometer để detect tầng
- **Software:** 3D map với multiple 2D layers
- **Use case:** Thư viện nhiều tầng

**4. Voice Control**
- **Hardware:** USB microphone
- **Software:** Speech-to-text (Google/Offline)
- **Commands:** "Go to section A", "Stop", "Return home"

**5. Object Recognition + Inventory**
- **Camera:** Sử dụng stereo camera hiện có
- **ML:** Train model nhận diện sách, kệ
- **Use case:** Tự động kiểm kê sách trong thư viện

**6. Collaborative SLAM (Multi-robot)**
- **Setup:** 2+ robots cùng map
- **Algorithm:** Distributed SLAM with map merging
- **Communication:** WiFi mesh network
- **Benefit:** Map nhanh hơn, cover diện tích lớn

#### 6.3.5. Tối ưu hóa

**1. Code optimization**
- **Hiện tại:** Pure Python → Slow
- **Nâng cấp:**
  - Cython cho critical loops
  - Numba JIT compilation
  - C++ extension cho performance-critical code

**2. CUDA acceleration**
- **Feature matching:** CUDA-accelerated ORB
- **Disparity computation:** GPU stereo matching
- **Neural network:** TensorRT optimization

**3. Power optimization**
- **Dynamic clocking:** Jetson power mode
  - Khi idle: 5W mode
  - Khi active: 10W mode
- **Sensor sleep:** Turn off cameras khi không cần

**4. Memory optimization**
- **Map compression:** Octree cho 3D points
- **Feature culling:** Xóa features cũ
- **Image downsampling:** Process ở 320x240 thay vì 640x480

### 6.4. Roadmap phát triển

#### Phase 1: Ổn định hệ thống hiện tại (1-2 tháng)
- [ ] Fix bugs hiện tại
- [ ] Improve documentation
- [ ] Add unit tests
- [ ] Battery monitoring
- [ ] Emergency stop button

#### Phase 2: Nâng cấp cảm biến (2-3 tháng)
- [ ] Thay sang encoder motor
- [ ] Thêm LIDAR 2D
- [ ] Thêm magnetometer
- [ ] IMX219 stereo camera

#### Phase 3: Nâng cấp thuật toán (3-4 tháng)
- [ ] ORB-SLAM3 full integration
- [ ] A* path planning
- [ ] ML-based obstacle detection
- [ ] Multi-sensor fusion upgrade

#### Phase 4: ROS Integration (2-3 tháng)
- [ ] Port code sang ROS
- [ ] Sử dụng ROS packages
- [ ] Simulation trong Gazebo
- [ ] RViz visualization

#### Phase 5: Tính năng nâng cao (3-6 tháng)
- [ ] Autonomous charging
- [ ] Cloud integration
- [ ] Voice control
- [ ] Object recognition
- [ ] Multi-robot collaboration

---

## 7. KẾT LUẬN

Robot tự hành lập bản đồ thư viện Ver 2.0 là một hệ thống hoàn chỉnh tích hợp nhiều công nghệ:
- **SLAM đa phương thức** (Visual + IMU)
- **Điều khiển di động** (Differential drive)
- **Phát hiện vật cản** thời gian thực
- **Khám phá tự động** với frontier-based exploration
- **Giao diện web** trực quan

Mặc dù còn một số hạn chế về phần cứng và thuật toán, hệ thống đã đạt được mục tiêu cơ bản: **Tự động khám phá và lập bản đồ môi trường trong nhà**.

Với roadmap phát triển rõ ràng, dự án có tiềm năng mở rộng thành một nền tảng robot dịch vụ đa năng cho thư viện, kho, và các môi trường trong nhà khác.

---

**Tài liệu được tạo:** {{current_date}}  
**Phiên bản:** 2.0  
**Tác giả:** Robot Development Team  
