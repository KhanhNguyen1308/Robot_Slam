# Hướng dẫn nhanh - Fix Camera Auto Settings

## Vấn đề
- 🔴 Camera tự động điều chỉnh exposure/white balance → 2 camera không khớp màu
- 🔴 Disparity map sai → Depth sai → Robot đâm vào tường

## Giải pháp ngay

### Bước 1: Chạy tuner tìm settings tốt nhất

```bash
cd /home/jetson/Robot_Slam/Ver2
python3 tune_camera_settings.py
```

**Phím điều khiển**:
- `Q` = Tối hơn (giảm exposure)
- `W` = Sáng hơn (tăng exposure) 
- `A` = Giảm gain (ít nhiễu hơn)
- `S` = Tăng gain (sáng hơn nhưng nhiễu hơn)
- `SPACE` = Copy settings
- `ESC` = Thoát

**Mục tiêu**: "Brightness diff" < 10 (màu xanh lá)

### Bước 2: Copy settings vào config

Mở `main.py`, tìm phần `'camera'` config, update:

```python
'camera': {
    ...
    'use_manual_settings': True,  # QUAN TRỌNG!
    'manual_exposure': -6,        # Từ tuner
    'manual_gain': 50,            # Từ tuner
}
```

### Bước 3: Chạy lại hệ thống

```bash
python3 main.py
```

## Kiểm tra kết quả

✅ 2 camera phải có màu sắc giống nhau  
✅ Disparity map phải rõ ràng (check web interface)  
✅ Robot phải dừng lại trước tường (không đâm!)  

## Điều chỉnh nhanh

### Quá tối:
```python
'manual_exposure': -4,  # Tăng (ít âm hơn)
'manual_gain': 60,      # Tăng
```

### Quá sáng:
```python
'manual_exposure': -8,  # Giảm (âm hơn)
'manual_gain': 40,      # Giảm
```

### Nhiễu:
```python
'manual_gain': 30,      # Giảm gain
'manual_exposure': -5,  # Tăng exposure bù
```

## Nếu vẫn lỗi

1. Check camera có hỗ trợ manual control không:
   ```bash
   v4l2-ctl -d /dev/video0 --list-ctrls
   v4l2-ctl -d /dev/video1 --list-ctrls
   ```

2. Set bằng v4l2-ctl nếu OpenCV không work:
   ```bash
   # Disable auto exposure
   v4l2-ctl -d /dev/video0 -c exposure_auto=1
   v4l2-ctl -d /dev/video1 -c exposure_auto=1
   
   # Set manual exposure (adjust value)
   v4l2-ctl -d /dev/video0 -c exposure_absolute=156
   v4l2-ctl -d /dev/video1 -c exposure_absolute=156
   ```

## Settings khuyến nghị theo môi trường

| Môi trường | Exposure | Gain |
|------------|----------|------|
| Trong nhà sáng | -6 | 40 |
| Trong nhà vừa | -5 | 50 |
| Trong nhà tối | -4 | 60 |
| Ngoài trời | -8 | 30 |

## Xem thêm

Chi tiết: [FIX_CAMERA_SETTINGS.md](FIX_CAMERA_SETTINGS.md)
