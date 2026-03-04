# FIX: Gear Ratio Sai - Robot Chỉ Quay 60° Thay Vì 360°

## ❌ Vấn Đề

Khi chạy rotation calibration test:
- **Lệnh**: Quay 360° (2π radians) trong 12.6 giây  
- **Thực tế**: Robot chỉ quay khoảng 60°
- **Sai số**: 360° / 60° = **6 lần**

## 🔍 Nguyên Nhân

**Gear ratio bị ngược trong config!**

### Hệ Thống Truyền Động Thực Tế

```
Motor ──[18 răng]──┐
                    │
                    ├─→ [38 răng]─┬─ Sprocket (20 răng, 66mm)
                                  │
                                  └─ Trục chung
```

- Motor có bánh răng 18 răng
- Trục trung gian có bánh răng 38 răng
- Sprocket 20 răng gắn cùng trục với bánh răng 38 răng

### Tỷ Số Truyền Đúng

**Gear Ratio = Số lần motor phải quay để wheel quay 1 vòng**

Motor 18 răng dẫn động intermediate 38 răng:
- Khi motor quay 1 vòng → bánh răng 18 răng quay 1 vòng
- Bánh răng 38 răng quay: 18/38 = 0.474 vòng
- Để bánh răng 38 răng quay 1 vòng → motor phải quay: **38/18 = 2.111 vòng**

**Vậy gear_ratio = 38/18 = 2.111** ✓

### Config Sai Trước Đây

```python
# WRONG!
'gear_ratio': 18/38,  # 0.474 (NGƯỢC!)
```

Comment nói "motor turns faster than sprocket" nhưng 18/38 = 0.474 < 1 có nghĩa motor quay **chậm hơn**, mâu thuẫn!

### Config Đúng

```python
# CORRECT!
'gear_ratio': 38/18,  # 2.111 (motor rotations per wheel rotation)
```

## 🔧 Steps Per Meter

### Công Thức Motor Controller

```python
steps_per_meter = (steps_per_rev × gear_ratio) / wheel_circumference
```

### So Sánh

**Với gear_ratio SAI (0.474)**:
```
steps_per_meter = (200 × 16 × 0.474) / (π × 0.066)
                = 1516.8 / 0.2073
                = 7310.5 steps/meter  ← Quá ít!
```

**Với gear_ratio ĐÚNG (2.111)**:
```
steps_per_meter = (200 × 16 × 2.111) / (π × 0.066)
                = 6755.2 / 0.2073
                = 32,597 steps/meter  ← Chính xác!
```

**Tỷ lệ**: 32597 / 7310.5 = **4.46 lần**

## 🎯 Giải Thích Tại Sao Quay 60° Thay Vì 360°

### Phân Tích

Với gear_ratio sai (quá nhỏ 4.46 lần):
- Motor không quay đủ số bước cần thiết
- Robot chỉ quay được: 360° / 6 = 60°

### Dự Đoán Sau Khi Sửa

Với gear_ratio đúng (tăng 4.46 lần):
- Robot sẽ quay: 60° × 4.46 ≈ **267°**
- Vẫn thiếu khoảng 93° để đủ 360°

**Nguyên nhân thiếu thêm**: `wheel_base` có thể chưa chính xác.

## ✅ Đã Sửa

### 1. main.py

```python
# Gear ratio configuration:
# Motor shaft: 18-tooth gear
# Intermediate shaft: 38-tooth gear (drives sprocket)
# Motor must rotate 38/18 = 2.111 times for wheel to rotate once
'gear_ratio': 38/18,  # 2.111 (motor rotations per wheel rotation)
```

### 2. calibrate_motors.py

```python
'gear_ratio': 38/18  # Motor rotations per wheel rotation (2.111)
```

### 3. calibrate_with_imu.py

```python
'gear_ratio': 38/18  # Motor rotations per wheel rotation (2.111)
```

## 🚀 Các Bước Tiếp Theo

### Bước 1: Test Lại với Gear Ratio Mới

Chạy lại rotation test:
```bash
python3 calibrate_motors.py
# Chọn option 2
# Bấm ENTER để dùng gear_ratio mới (38/18 = 2.111)
```

**Dự đoán**: Robot sẽ quay được ~267° (tốt hơn nhiều so với 60°)

### Bước 2: Hiệu Chỉnh Chính Xác với IMU

Để đo chính xác góc quay thực tế:
```bash
python3 calibrate_with_imu.py
# Chọn option 2
```

Tool này sẽ:
- Dùng MPU6050 đo góc quay **thực tế**
- Tự động tính wheel_base chính xác
- Loại bỏ sai số do trượt xích, ma sát, v.v.

### Bước 3: Hiệu Chỉnh Linear Motion

Sau khi có gear_ratio đúng, test khoảng cách:
```bash
python3 calibrate_motors.py
# Chọn option 1
```

Nếu robot đi đúng ~1 mét (±5cm) → **gear_ratio 38/18 chính xác!**

Nếu vẫn sai → có thể đường kính sprocket không đúng 66mm.

## 🔬 Kiểm Tra Thêm

### Nếu Robot Vẫn Không Quay Đúng 360°

Có thể kiểm tra:

1. **Wheel diameter** - Đo chính xác đường kính sprocket
   ```python
   # Đo chu vi thực tế
   circumference = actual_measured_circumference_in_meters
   wheel_diameter = circumference / math.pi
   ```

2. **Track slippage** - Xích có thể trượt trên sprocket
   - Kiểm tra độ căng xích
   - Đảm bảo sprocket không bị mòn

3. **Motor steps** - Xác nhận motor là 200 steps/rev
   - Kiểm tra datasheet motor
   - Test xem 200 steps có quay đúng 1 vòng không

4. **Microstepping** - Xác nhận đang dùng 1/16
   - Kiểm tra MS1, MS2, MS3 pins
   - Đọc log xác nhận "Microstepping: 1/16"

## 📊 Kết Quả Mong Đợi

### Sau Khi Sửa Gear Ratio (38/18)

| Test | Trước (18/38) | Sau (38/18) | Cải thiện |
|------|---------------|-------------|-----------|
| Rotation 360° | 60° | ~267° | 4.5x |
| Linear 1.0m | ~0.225m | ~1.0m | 4.5x |
| Steps/meter | 7310 | 32597 | 4.5x |

### Sau Khi Hiệu Chỉnh wheel_base với IMU

| Test | Trước | Sau | Độ chính xác |
|------|-------|-----|--------------|
| Rotation 360° | 267° | 360° ± 2° | ✓ |
| Rotation 90° | 67° | 90° ± 1° | ✓ |
| Rotation -180° | -133° | -180° ± 2° | ✓ |

## 📝 Tóm Tắt

### Lỗi Chính

**Gear ratio bị ngược**: 18/38 thay vì 38/18

### Hậu Quả

- Robot di chuyển **thiếu 4.5 lần** so với lệnh
- Quay 60° thay vì 360°
- Đi 0.225m thay vì 1.0m
- SLAM/odometry hoàn toàn sai

### Giải Pháp

1. ✅ Sửa `gear_ratio: 38/18` trong [main.py](main.py#L408)
2. 🔄 Test lại với [calibrate_motors.py](calibrate_motors.py)
3. 🎯 Hiệu chỉnh chính xác với [calibrate_with_imu.py](calibrate_with_imu.py)

---

**Ghi chú**: Sau khi sửa, nhớ restart hệ thống để áp dụng config mới!
