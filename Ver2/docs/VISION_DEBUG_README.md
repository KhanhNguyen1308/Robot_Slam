# Vision Processing Debug Tool

## 📝 Mô Tả

Web interface để visualize từng bước xử lý ảnh trong stereo vision pipeline:

1. **Camera Capture** - Lấy ảnh từ 2 camera stereo
2. **Grayscale Conversion** - Chuyển đổi sang ảnh xám
3. **Disparity & Depth** - Tính toán bản đồ độ sâu
4. **FAST Feature Detection** - Phát hiện corner points
5. **ORB Descriptors** - Mô tả đặc trưng (BRIEF + orientation)
6. **Hamming Matching** - So khớp features giữa 2 ảnh

## 🚀 Cách Sử Dụng

### Khởi động server

```bash
python3 vision_debug_server.py
```

Server sẽ chạy trên: `http://0.0.0.0:5001`

### Truy cập giao diện web

Mở trình duyệt và vào:
```
http://<jetson-ip>:5001
```

Hoặc trên chính Jetson:
```
http://localhost:5001
```

### Dừng server

Nhấn `Ctrl+C` trong terminal

## 📊 Giao Diện

### Performance Stats
- **FPS**: Tốc độ xử lý (frames per second)
- **Process Time**: Thời gian xử lý 1 frame (ms)
- **Left/Right Keypoints**: Số lượng features phát hiện được
- **Matches**: Số lượng feature matches giữa 2 ảnh

### Pipeline Steps

Mỗi bước hiển thị:
- Số thứ tự (1-6)
- Tiêu đề và mô tả chi tiết
- Hình ảnh kết quả

## 🔧 Cấu Hình

Để thay đổi camera IDs hoặc resolution, sửa trong `vision_debug_server.py`:

```python
debugger = VisionDebugger(
    left_cam_id=1,    # Camera trái
    right_cam_id=0,   # Camera phải
    width=640,        # Độ rộng
    height=480        # Độ cao
)
```

### Điều chỉnh thuật toán

**FAST threshold** (độ nhạy phát hiện corner):
```python
self.fast = cv2.FastFeatureDetector_create(threshold=25)  # Giảm = nhiều features hơn
```

**ORB features** (số lượng features tối đa):
```python
self.orb = cv2.ORB_create(nfeatures=500)  # Tăng = nhiều features hơn
```

**Stereo matching**:
```python
self.stereo = cv2.StereoBM_create(
    numDisparities=16*5,  # Phải là bội số của 16
    blockSize=15          # Kích thước block (lẻ, 5-21)
)
```

## 📚 Thuật Toán

### FAST (Features from Accelerated Segment Test)
- Phát hiện corner bằng cách kiểm tra 16 pixel xung quanh
- Rất nhanh: O(1) per pixel
- Sử dụng threshold để quyết định pixel có phải corner không

### BRIEF (Binary Robust Independent Elementary Features)
- Mô tả feature bằng binary string (256-512 bits)
- So sánh cường độ sáng của các cặp pixel
- Compact và nhanh để so khớp

### ORB (Oriented FAST and Rotated BRIEF)
- Kết hợp FAST detection với BRIEF description
- Thêm orientation (hướng) để rotation-invariant
- Scale-invariant với image pyramid
- Free và open-source (không như SIFT/SURF có patent)

### Hamming Distance
- Đo khoảng cách giữa 2 binary descriptors
- Đếm số bit khác nhau: `XOR` rồi đếm số bit `1`
- Rất nhanh với CPU instruction `POPCNT`
- Phù hợp cho binary descriptors (ORB, BRIEF, BRISK)

### Stereo Matching (StereoBM)
- Block Matching algorithm
- Tìm correspondence giữa left và right image
- Tính disparity: `d = x_left - x_right`
- Depth: `Z = (f × B) / d` (f=focal length, B=baseline)

## ⚙️ Dependencies

- OpenCV (`cv2`)
- NumPy
- Flask

Đã được cài đặt trong main system.

## 🔍 Troubleshooting

### Không mở được camera
```
Error: Failed to open cameras!
```
→ Kiểm tra camera IDs (thử 0, 1, 2...)
→ Kiểm tra quyền truy cập: `ls -l /dev/video*`

### Web không hiển thị ảnh
→ Kiểm tra firewall: `sudo ufw allow 5001`
→ Kiểm tra console browser (F12) xem có lỗi không

### FPS thấp
→ Giảm resolution (320×240)
→ Giảm số features: `nfeatures=200`
→ Tăng update interval trong HTML: `setInterval(updateImages, 200)` (200ms)

## 📝 Notes

- **Độc lập hoàn toàn** - không ảnh hưởng đến main.py hay các script khác
- Chạy trên **port 5001** (main web server dùng port 5000)
- Auto-refresh mỗi 100ms (~10 FPS display)
- Chỉ hiển thị top 20 matches để tránh cluttered

## 🎯 Use Cases

- **Debug stereo camera setup** - Kiểm tra alignment, calibration
- **Tune feature detection** - Điều chỉnh threshold, số features
- **Education** - Học cách stereo vision hoạt động
- **Performance analysis** - Đo FPS, process time

## 🔗 Related Files

- `vision_debug_server.py` - Main server script
- `templates/vision_debug.html` - Web interface
- `stereo_camera.py` - Production stereo camera code (tham khảo)
- `orb_slam3_wrapper.py` - SLAM system sử dụng ORB features
