# Hướng Dẫn Hiệu Chỉnh Motor - Calibration Guide

## Cấu Hình Hệ Thống Truyền Động

### Thông Số Phần Cứng
- **Động cơ**: Stepper motor 200 steps/rev, microstepping 1/16
- **Bánh răng motor**: 18 răng (gắn trục động cơ)
- **Bánh răng trung gian**: 38 răng (cùng trục với sprocket)
- **Sprocket xích**: 20 răng, đường kính 66mm
- **Khoảng cách 2 xích**: 284.5mm (center to center)
- **Pitch xích**: 15mm giữa các mắt xích

### Tỷ Số Truyền (Gear Ratio)
```
Gear Ratio = Số răng motor / Số răng trung gian
           = 18 / 38
           = 0.474
```

**Ý nghĩa**: Motor quay nhanh hơn sprocket. Khi motor quay 38 vòng thì sprocket chỉ quay 18 vòng.

### Công Thức Tính Toán

**Chu vi sprocket**:
```
C = π × D = 3.14159 × 0.066m = 0.2073m
```

**Số bước motor trên 1 mét**:
```
Steps/meter = (Steps/rev × Microsteps × Gear_ratio) / Wheel_circumference
            = (200 × 16 × 0.474) / 0.2073
            = 7299 steps/meter
```

## Quy Trình Hiệu Chỉnh

### Bước 1: Hiệu Chỉnh Khoảng Cách (Linear Calibration)

**Mục đích**: Đảm bảo robot di chuyển đúng khoảng cách theo lệnh

**Công cụ cần thiết**:
- Thước dây đo (measuring tape)
- Băng keo đánh dấu vị trí xuất phát
- Khoảng trống ít nhất 2 mét

**Thực hiện**:
```bash
python3 calibrate_motors.py
# Chọn option 1 hoặc 3
```

**Các bước**:
1. Script sẽ ra lệnh robot đi thẳng 1 mét (với tốc độ 0.2m/s trong 5 giây)
2. Dùng thước đo khoảng cách **thực tế** robot đã di chuyển
3. Nhập khoảng cách đo được
4. Script sẽ tính toán `gear_ratio` mới

**Ví dụ**:
- Lệnh: Đi 1.000m
- Thực tế: Robot chỉ đi 0.850m (thiếu 15cm)
- → Gear ratio mới = 0.474 × (1.000/0.850) = 0.558

**Cập nhật** trong `main.py`:
```python
'gear_ratio': 0.558,  # Thay giá trị mới vào đây
```

### Bước 2: Hiệu Chỉnh Góc Quay (Rotation Calibration)

**Mục đích**: Đảm bảo robot quay đúng góc theo lệnh

**Công cụ cần thiết**:
- Băng keo hoặc bút đánh dấu hướng ban đầu
- Khoảng trống ít nhất 1 mét xung quanh robot

**Thực hiện**:
```bash
python3 calibrate_motors.py
# Chọn option 2
# Nhập gear_ratio đã hiệu chỉnh ở bước 1
```

**Các bước**:
1. Đánh dấu hướng robot hiện tại (vẽ mũi tên trên sàn)
2. Script sẽ quay robot 360° (với tốc độ 0.5rad/s trong ~12.6 giây)
3. Quan sát robot có quay về đúng hướng ban đầu không

**Phân tích kết quả**:

- **✓ Đúng hướng** (sai số < 5°): `wheel_base` đã chính xác, giữ nguyên 0.2845m
  
- **↻ Quay thừa** (vượt quá hướng ban đầu): `wheel_base` hiện tại quá lớn
  - Ví dụ: Quay thừa 20° → wheel_base mới = 0.2845 × (360-20)/360 = 0.2687m
  
- **↺ Quay thiếu** (chưa về hướng ban đầu): `wheel_base` hiện tại quá nhỏ
  - Ví dụ: Thiếu 15° → wheel_base mới = 0.2845 × (360+15)/360 = 0.2963m

**Cập nhật** trong `main.py`:
```python
'wheel_base': 0.2963,  # Thay giá trị mới vào đây
```

## Lưu Ý Quan Trọng

### ⚠️ An Toàn
1. **Luôn giữ khoảng trống** khi chạy calibration
2. **Sẵn sàng tắt nguồn** nếu robot hoạt động bất thường
3. **Không để tay gần xích** khi motor đang chạy

### 🔧 Độ Chính Xác
1. **Đo nhiều lần**: Chạy test 3 lần và lấy trung bình
2. **Kiểm tra bề mặt**: Đảm bảo sàn phẳng, không trơn trượt
3. **Pin đầy**: Đảm bảo nguồn điện ổn định (motor yếu sẽ cho kết quả sai)

### 🔄 Quy Trình Lặp Lại
Sau khi cập nhật config:
1. Khởi động lại `main.py`
2. Kiểm tra lại bằng calibration script
3. Nếu vẫn sai > 5%, lặp lại quy trình

## Giải Thích Kỹ Thuật

### Tại sao cần hiệu chỉnh?

**Gear ratio** ảnh hưởng đến khoảng cách:
- Nếu quá nhỏ → Robot đi **thiếu** (ví dụ: lệnh 1m nhưng chỉ đi 0.8m)
- Nếu quá lớn → Robot đi **thừa** (ví dụ: lệnh 1m nhưng đi 1.2m)

**Wheel base** ảnh hưởng đến góc quay:
- Nếu quá nhỏ → Robot quay **thiếu** (lệnh 90° nhưng chỉ quay 80°)
- Nếu quá lớn → Robot quay **thừa** (lệnh 90° nhưng quay 100°)

### Tại sao không dùng giá trị lý thuyết?

Các yếu tố gây sai số:
- **Đường kính sprocket thực tế** ≠ 66mm chính xác (dung sai sản xuất)
- **Trượt xích**: Xích có thể trượt nhẹ trên sprocket
- **Độ căng xích**: Xích lỏng/chặt ảnh hưởng pitch thực tế
- **Ma sát sàn**: Bề mặt khác nhau cho kết quả khác nhau
- **Trọng lượng**: Camera, pin làm thay đổi điểm tiếp xúc

→ **Phải hiệu chỉnh dựa trên đo đạc thực tế!**

## Troubleshooting

### Robot không di chuyển
```bash
# Kiểm tra motor enable
python3 test_motor_simple.py
```

### Robot di chuyển quá nhanh/chậm
- Kiểm tra nguồn điện (cần ít nhất 12V 2A)
- Kiểm tra microstepping setting (phải là 16)

### Kết quả không ổn định
- Kiểm tra xích có bị lỏng không
- Kiểm tra sprocket có bị mòn không
- Thử trên bề mặt khác (gỗ/lát gạch)

### Robot quay không tròn
- Kiểm tra 2 bên xích căng đều nhau
- Kiểm tra motor 2 bên hoạt động giống nhau
- Đo lại wheel_base thực tế bằng thước

## Tham Khảo

**File cấu hình**: `main.py` (dòng 360-410)  
**Test motor**: `test_motor_simple.py`  
**Hiệu chỉnh**: `calibrate_motors.py`

**Hotline**: Kiểm tra log trong `robot.log` nếu có lỗi
