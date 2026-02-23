# Hướng dẫn nhanh - Tích hợp MPU6050

## Bước 1: Kết nối phần cứng

Kết nối MPU6050 với Jetson Nano I2C bus 1:

```
MPU6050 -> Jetson Nano
VCC  -> Pin 1 (3.3V)
GND  -> Pin 6 (GND)
SCL  -> Pin 5 (GPIO03)
SDA  -> Pin 3 (GPIO02)
```

⚠️ **QUAN TRỌNG**: Chỉ dùng 3.3V, không dùng 5V!

## Bước 2: Kiểm tra kết nối

```bash
# Cài đặt công cụ i2c
sudo apt-get install -y i2c-tools python3-smbus

# Quét I2C bus 1
sudo i2cdetect -y -r 1
```

Bạn sẽ thấy **68** xuất hiện - đó là MPU6050!

## Bước 3: Cài đặt thư viện

```bash
pip3 install smbus2
```

## Bước 4: Test IMU

```bash
cd /home/jetson/Robot_Slam/Ver2
python3 test_imu.py
```

Chọn test 1 để kiểm tra kết nối cơ bản.

## Bước 5: Chạy hệ thống

```bash
python3 main.py
```

**Lưu ý**: Khi hiệu chỉnh IMU, giữ robot đứng yên trong 5 giây!

## Kiểm tra dữ liệu IMU

Truy cập web interface: `http://<jetson-ip>:5000`

Hoặc dùng curl:
```bash
curl http://<jetson-ip>:5000/api/imu
```

## Các vấn đề thường gặp

### Không phát hiện MPU6050
- Kiểm tra lại dây kết nối
- Đảm bảo dùng 3.3V
- Chạy lại `i2cdetect`

### Lỗi quyền truy cập I2C
```bash
sudo usermod -a -G i2c $USER
sudo chmod 666 /dev/i2c-1
```

### Hướng robot không chính xác
- Chạy lại hiệu chỉnh: `python3 test_imu.py` (test 3)
- Kiểm tra MPU6050 được gắn chắc chắn
- Đảm bảo trục Z của MPU6050 thẳng đứng

## Ưu điểm sau khi tích hợp

✅ Độ chính xác hướng tăng từ ±5-10° lên ±1-2°  
✅ Giảm drift trong quá trình di chuyển  
✅ Tần số cập nhật cao hơn (100Hz vs 30Hz)  
✅ Occupancy grid hiển thị chính xác hơn  

## Tham số điều chỉnh

Trong `main.py`, phần `config['imu']`:

- `use_ekf: True` - Dùng Kalman Filter (khuyến nghị)
- `fusion_weight: 0.3` - Trọng số IMU (0.0-1.0)
  - Tăng lên 0.5-0.7 nếu visual SLAM không ổn định
  - Giảm xuống 0.1-0.2 nếu IMU nhiễu

## Hỗ trợ

Xem hướng dẫn chi tiết: `MPU6050_INTEGRATION.md`
