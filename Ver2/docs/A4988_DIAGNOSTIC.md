# CHẨN ĐOÁN VÀ SỬA LỖI A4988 + NEMA 17

## Triệu chứng hiện tại:
- Động cơ kêu "è è", "cạch cạch"
- Rung mạnh nhưng trục không quay hoặc quay giật cục
- Bánh răng dính chặt vào trục

## NGUYÊN NHÂN CHÍNH:

### ⚠️ VẤN ĐỀ 1: MICROSTEPPING KHÔNG KHỚP (QUAN TRỌNG NHẤT!)
**Code cấu hình 16x microstepping nhưng chân MS1, MS2, MS3 không kết nối!**

Trong `main.py` line 298-300:
```python
'left_ms1_pin': None,  # <-- KHÔNG KẾT NỐI!
'left_ms2_pin': None,  # <-- KHÔNG KẾT NỐI!
'left_ms3_pin': None,  # <-- KHÔNG KẾT NỐI!
```

Khi MS pins không kết nối (floating):
- A4988 hardware có thể ở chế độ FULL STEP hoặc ngẫu nhiên
- Software tính toán cho 16x microstepping (3200 steps/rev)
- Hardware chỉ cần 200 steps/rev (Full step)
- **Kết quả: Motor nhận quá nhiều xung → rung giật!**

### 📋 BẢNG MICROSTEPPING A4988:
```
MS1 | MS2 | MS3 | Microstepping | Steps/Rev (NEMA 17)
----|-----|-----|---------------|--------------------
 L  |  L  |  L  | Full Step     | 200
 H  |  L  |  L  | Half Step     | 400
 L  |  H  |  L  | 1/4 Step      | 800
 H  |  H  |  L  | 1/8 Step      | 1,600
 H  |  H  |  H  | 1/16 Step     | 3,200
```

### ⚠️ VẤN ĐỀ 2: DÒNG ĐIỆN A4988 (VREF)
NEMA 17 thường cần 1.0-1.7A, nhưng VREF chưa được điều chỉnh!

**Công thức tính VREF cho A4988:**
```
VREF = Current_Limit / 2.5
```

Ví dụ cho NEMA 17 rated 1.5A:
- Current limit cần đặt = 1.5A × 0.7 = 1.05A (70% để an toàn)
- VREF = 1.05 / 2.5 = 0.42V

**CÁCH ĐO VÀ ĐIỀU CHỈNH VREF:**
1. Bật nguồn 12V cho A4988 (CHƯA KẾT NỐI MOTOR)
2. Dùng đồng hồ vạn năng (multimeter) đo DC voltage
3. Chân đen (COM) → GND của A4988
4. Chân đỏ (V) → đỉnh con đinh vít potentiometer trên A4988
5. Vặn đinh vít từ từ để đạt VREF = 0.4-0.5V
6. VẶN THEO CHIỀU KIM ĐỒNG HỒ = TĂNG, NGƯỢC CHIỀU = GIẢM

### ⚠️ VẤN ĐỀ 3: TỐC ĐỘ XUNG QUÁ NHANH
Code sử dụng `time.sleep()` không đủ chính xác ở tốc độ cao.

A4988 yêu cầu:
- Minimum pulse width: 1μs (code dùng 5μs là đủ)
- Maximum step frequency: ~15-20 kHz

### ⚠️ VẤN ĐỀ 4: KIỂM TRA WIRING
Sơ đồ kết nối A4988 đúng:

**Motor NEMA 17:**
- Có 4 dây: 2 coils (A và B)
- Mỗi coil có 2 dây
- Dùng đồng hồ đo điện trở: 2 dây cùng coil sẽ có điện trở ~1-4Ω

**Kết nối A4988:**
```
Motor Wire    A4988 Pin
---------     ---------
Coil A1   →   1B (hoặc 2B)
Coil A2   →   1A (hoặc 2A)
Coil B1   →   2B (hoặc 1B)
Coil B2   →   2A (hoặc 1A)
```

**Nguồn điện:**
- VMOT: 12V (8-35V range)
- Capacitor 100μF gần A4988 (giảm nhiễu)
- GND chung giữa Jetson và A4988

---

## 🔧 GIẢI PHÁP:

### GIẢI PHÁP 1A: KẾT NỐI MS PINS (KHUYÊN DÙNG)
Kết nối 3 chân MS1, MS2, MS3 từ A4988 vào Jetson GPIO:

**Ví dụ cho motor trái:**
```python
'left_ms1_pin': 11,   # Pin 11 (GPIO 17)
'left_ms2_pin': 13,   # Pin 13 (GPIO 27)
'left_ms3_pin': 15,   # Pin 15 (GPIO 22)
```

**Ví dụ cho motor phải:**
```python
'right_ms1_pin': 16,  # Pin 16 (GPIO 23)
'right_ms2_pin': 18,  # Pin 18 (GPIO 24)
'right_ms3_pin': 22,  # Pin 22 (GPIO 25)
```

### GIẢI PHÁP 1B: NỐI MS PINS TRỰC TIẾP (ĐƠN GIẢN HƠN)
Nếu không muốn dùng Jetson GPIO điều khiển MS:

**Cho Full Step (mượt nhất, mạnh nhất):**
- Nối MS1, MS2, MS3 → GND

**Cho 1/16 Step (êm nhất, chính xác nhất):**
- Nối MS1, MS2, MS3 → VDD (3.3V hoặc 5V của A4988)

**Sau đó sửa code config:**
```python
'microsteps': 1,  # Nếu nối MS → GND
# HOẶC
'microsteps': 16, # Nếu nối MS → VDD
```

### GIẢI PHÁP 2: ĐIỀU CHỈNH VREF
1. Kiểm tra rating của NEMA 17 (thường ghi trên motor)
2. Đặt current limit = rated current × 0.7
3. Tính VREF = current limit / 2.5
4. Điều chỉnh potentiometer trên A4988

### GIẢI PHÁP 3: GIẢM TỐC ĐỘ TEST
Sửa trong `main.py`:
```python
'max_linear_speed': 0.1,   # Giảm từ 0.3 xuống 0.1 m/s
'max_angular_speed': 0.5   # Giảm từ 1.5 xuống 0.5 rad/s
```

---

## 📝 BƯỚC KIỂM TRA TỪNG BƯỚC:

### BƯỚC 1: KIỂM TRA PHẦN CỨNG
```bash
# Tắt chương trình nếu đang chạy
# Kiểm tra:
□ Nguồn 12V đã cắm vào A4988 VMOT chưa?
□ GND của Jetson và A4988 đã nối chung chưa?
□ 4 dây motor đã cắm đúng vào 1A, 1B, 2A, 2B chưa?
□ Enable pin có đang LOW không? (Motor phải enabled)
□ Có tụ 100μF gần chân VMOT của A4988 không?
```

### BƯỚC 2: ĐO VÀ ĐIỀU CHỈNH VREF
```bash
# Bật nguồn 12V cho A4988
# Đo VREF với multimeter
# Điều chỉnh về 0.4-0.5V cho motor 1.5A rated
```

### BƯỚC 3: CẤU HÌNH MICROSTEPPING
**Option A: Nối cứng MS pins**
```bash
# Nối MS1, MS2, MS3 tất cả vào GND → Full step
# Hoặc nối tất cả vào VDD → 1/16 step
```

**Option B: Điều khiển qua GPIO**
```bash
# Nối MS1, MS2, MS3 vào các GPIO pins
# Code sẽ tự động cấu hình
```

### BƯỚC 4: TEST MOTOR ĐƠN GIẢN
Tạo file test đơn giản:

```python
# test_motor_simple.py
import Jetson.GPIO as GPIO
import time

# Cấu hình
STEP_PIN = 33
DIR_PIN = 35
ENABLE_PIN = 37

GPIO.setmode(GPIO.BOARD)
GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH)

# Enable motor
GPIO.output(ENABLE_PIN, GPIO.LOW)
print("Motor enabled")

# Chọn hướng
GPIO.output(DIR_PIN, GPIO.HIGH)
print("Direction: Forward")

# Quay 200 bước (1 vòng full step)
print("Stepping 200 steps slowly...")
for i in range(200):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.001)  # 1ms HIGH
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(0.001)  # 1ms LOW
    if i % 50 == 0:
        print(f"Step {i}/200")

print("Done!")
GPIO.output(ENABLE_PIN, GPIO.HIGH)
GPIO.cleanup()
```

Chạy test:
```bash
sudo python3 test_motor_simple.py
```

**Kết quả mong đợi:**
- Motor quay 1 vòng (360°) trong 0.4 giây
- Không có tiếng "è è" hay rung giật
- Quay mượt và êm

### BƯỚC 5: KIỂM TRA WIRING MOTOR
Nếu vẫn không quay, kiểm tra coil pairing:

```python
# check_motor_coils.py
import Jetson.GPIO as GPIO
import time

STEP_PIN = 33
DIR_PIN = 35
ENABLE_PIN = 37
# Thêm MS pins nếu có
MS1_PIN = None
MS2_PIN = None
MS3_PIN = None

GPIO.setmode(GPIO.BOARD)
GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH)

if MS1_PIN:
    GPIO.setup(MS1_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MS2_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MS3_PIN, GPIO.OUT, initial=GPIO.LOW)
    
# Enable motor
GPIO.output(ENABLE_PIN, GPIO.LOW)

print("Testing motor coil configuration...")
print("Motor should rotate smoothly")
print("Press Ctrl+C to stop")

try:
    while True:
        # Step với tốc độ chậm
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.002)
except KeyboardInterrupt:
    print("\nStopped")
    
GPIO.output(ENABLE_PIN, GPIO.HIGH)
GPIO.cleanup()
```

---

## ✅ CHECKLIST HOÀN CHỈNH:

### Phần cứng:
- [ ] Nguồn 12V đấu đúng vào VMOT và GND của A4988
- [ ] GND Jetson và GND A4988 nối chung
- [ ] Tụ điện 100μF gắn gần A4988
- [ ] 4 dây motor đấu vào 1A, 1B, 2A, 2B
- [ ] VREF đã điều chỉnh về 0.4-0.5V (cho motor 1.5A)
- [ ] MS1, MS2, MS3 đã nối hoặc vào GND hoặc vào VDD (không để floating!)

### Kết nối Jetson → A4988:
- [ ] STEP pin nối đúng (Pin 33 → A4988 STEP)
- [ ] DIR pin nối đúng (Pin 35 → A4988 DIR)
- [ ] ENABLE pin nối đúng (Pin 37 → A4988 EN)

### Software:
- [ ] Config microsteps khớp với hardware MS pins
- [ ] Tốc độ giảm xuống để test (0.1 m/s)
- [ ] Chạy test_motor_simple.py thành công

---

## 🎯 HÀNH ĐỘNG NGAY:

**QUAN TRỌNG NHẤT - LÀM NGAY:**

1. **Nối MS pins vào GND hoặc VDD** (không để floating!)
   - KHUYẾN NGHỊ: Nối MS1, MS2, MS3 → VDD để dùng 1/16 step
   
2. **Đo và điều chỉnh VREF = 0.4-0.5V**

3. **Chạy test_motor_simple.py** để kiểm tra

4. **Nếu vẫn không quay:** Đảo 2 dây của một coil (ví dụ đổi chỗ 1A và 1B)

---

## ⚡ CẤU HÌNH KHUYẾN NGHỊ:

Sau khi sửa phần cứng, sửa file `main.py`:

```python
'motor': {
    # ... pins khác ...
    'microsteps': 16,           # Đặt 16 nếu MS pins nối VDD
    'max_linear_speed': 0.2,    # Giảm để an toàn ban đầu
    'max_angular_speed': 1.0    # Giảm để an toàn ban đầu
}
```

Nếu muốn dùng GPIO điều khiển MS pins (linh hoạt hơn):
```python
'motor': {
    # ... pins khác ...
    'left_ms1_pin': 11,   # Pin 11 (GPIO 17)
    'left_ms2_pin': 13,   # Pin 13 (GPIO 27)
    'left_ms3_pin': 15,   # Pin 15 (GPIO 22)
    'right_ms1_pin': 16,  # Pin 16 (GPIO 23)
    'right_ms2_pin': 18,  # Pin 18 (GPIO 24)
    'right_ms3_pin': 22,  # Pin 22 (GPIO 25)
    'microsteps': 16,
}
```

**HÃY BẮT ĐẦU VỚI GIẢI PHÁP 1B (nối MS pins cứng) - ĐƠN GIẢN NHẤT!**
