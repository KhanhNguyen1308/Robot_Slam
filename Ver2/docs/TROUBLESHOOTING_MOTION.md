# TROUBLESHOOTING: Robot di chuyển loạn / False Motion

## Vấn Đề: Robot "nhảy" hoặc di chuyển loạn trên map khi đứng yên

### Nguyên nhân chính

#### 1. ⚠️ **BASELINE SAI** (Quan trọng nhất!)

**Triệu chứng:**
- Robot đứng yên nhưng vị trí trên map nhảy lung tung
- Depth measurements không chính xác
- SLAM tính toán sai khoảng cách và chuyển động

**Nguyên nhân:**
- Config baseline khác với hardware thực tế
- Calibration file có baseline sai
- Default baseline (60mm) khác với hardware (140mm)

**Giải pháp:**
```bash
# 1. Kiểm tra calibration
python3 check_calibration.py

# 2. Đo baseline thực tế (khoảng cách giữa 2 camera lens centers)
# Hardware của bạn: 140mm (14cm)

# 3. Update config trong main.py
'camera': {
    'stereo_baseline': 0.14,  # ← PHỤ THUỘC HARDWARE!
}

# 4. Nếu có calibration.npz, verify baseline trong file match hardware
```

**Công thức depth:**
```python
depth = (focal_length × baseline) / disparity
```
Nếu baseline sai → depth sai → SLAM tính motion sai!

---

#### 2. **Stereo Calibration Kém**

**Triệu chứng:**
- Disparity map có nhiều noise/holes
- Depth không ổn định
- Camera rectification không tốt

**Kiểm tra:**
```bash
python3 check_calibration.py
# Xem RMS error:
#   < 0.5 pixels: Excellent
#   < 1.0 pixels: Good
#   > 2.0 pixels: Poor - cần recalibrate!
```

**Giải pháp:**
1. Re-calibrate với nhiều ảnh hơn (20-30 ảnh)
2. Dùng chessboard pattern lớn và rõ ràng
3. Chụp từ nhiều góc độ và khoảng cách khác nhau
4. Đảm bảo cameras mounted chắc chắn (không rung)

---

#### 3. **Feature Matching Noise**

**Triệu chứng:**
- Nhiều "small jumps" liên tục
- Motion detection từ static scene

**Nguyên nhân:**
- ORB features unstable
- Poor lighting
- Camera auto-exposure thay đổi

**Giải pháp đã implement:**
```python
# SimpleVisualSLAM đã có motion filtering:
self.min_translation_threshold = 0.005  # 5mm minimum
self.max_translation_per_frame = 0.5   # 50cm max

# Motion < 5mm bị filter ra (noise)
# Motion > 50cm bị reject (lỗi)
```

**Tune parameters nếu cần:**
```python
# orb_slam3_wrapper.py, SimpleVisualSLAM.__init__()
self.min_translation_threshold = 0.01  # Tăng để filter nhiều hơn
```

---

#### 4. **Camera Frame Sync Issues**

**Triệu chứng:**
- Disparity map có "ghosting"
- Temporal jitter

**Kiểm tra:**
```python
# Trong stereo_camera.py
# Verify left/right frames được capture đồng thời
ret_l, frame_l = self.left_cam.read()
ret_r, frame_r = self.right_cam.read()
```

**Giải pháp:**
- Dùng hardware-triggered cameras nếu có thể
- Giảm FPS để đảm bảo sync (30fps → 15fps)
- Check USB bandwidth (2 cameras cùng USB bus?)

---

#### 5. **Lighting/Exposure Changes**

**Triệu chứng:**
- Motion detection khi lighting thay đổi
- False features khi có shadows

**Giải pháp:**
```python
# Fix camera exposure (disable auto-exposure)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
cap.set(cv2.CAP_PROP_EXPOSURE, -5)         # Fixed value
```

---

## Quy Trình Debug

### Bước 1: Verify Baseline
```bash
cd /home/jetson/Robot_Slam/Ver2

# Check calibration
python3 check_calibration.py

# Kết quả mong đợi:
# ⭐ STEREO BASELINE: 140.00mm (0.1400m)
# ✓ MATCH! Calibration matches hardware
```

**Nếu baseline SAI:**
```python
# Edit main.py
'camera': {
    'stereo_baseline': 0.14,  # ← Update giá trị đúng!
}
```

### Bước 2: Check SLAM Logs
```bash
# Xem robot.log
tail -f robot.log

# Tìm các dòng:
# "Stereo baseline in use: 140.0mm"  ← Verify đúng
# "Filtering small motion: X.XXmm"   ← Motion filtering hoạt động
# "Motion detected: X.Xcm"           ← Real motions
```

### Bước 3: Monitor Motion Filtering
```python
# Trong main loop, check stats
slam_stats = slam_system.get_stats()
# Nếu frames_without_motion cao → robot đang đứng yên (tốt!)
```

### Bước 4: Test Static Scene
```bash
# Chạy robot với camera nhìn vào scene tĩnh
python3 main.py

# Observe:
# - Position trên map có nhảy không?
# - Log có "Motion detected" không? (không nên có nếu đứng yên)
```

---

## Quick Fixes

### Fix 1: Update Baseline (QUAN TRỌNG NHẤT!)
```python
# main.py, trong load_config()
'camera': {
    'stereo_baseline': 0.14,  # ← 140mm = 0.14m
}
```

### Fix 2: Increase Motion Threshold
```python
# orb_slam3_wrapper.py, SimpleVisualSLAM.__init__()
self.min_translation_threshold = 0.01  # 10mm thay vì 5mm
```

### Fix 3: Reduce SLAM Update Rate
```python
# main.py, _main_loop()
time.sleep(0.05)  # 20Hz thay vì 100Hz
```

### Fix 4: Disable SLAM khi đứng yên
```python
# Thêm vào main_loop:
current_velocity = motor_controller.current_linear
if abs(current_velocity) < 0.01:  # Đứng yên
    continue  # Skip SLAM update
```

---

## Verification Checklist

Sau khi fix, verify:

- [ ] **Baseline đúng**: `check_calibration.py` shows 140mm
- [ ] **Config match**: `main.py` có `stereo_baseline: 0.14`
- [ ] **Log confirm**: Robot.log shows "baseline in use: 140.0mm"
- [ ] **Static test**: Robot đứng yên → position không nhảy
- [ ] **Motion filtering**: Log shows "Filtering small motion"
- [ ] **Real motion**: Di chuyển robot → position update đúng

---

## Expected Behavior

### ✅ ĐÚNG:
```
# Robot đứng yên
[INFO] Filtering small motion: 2.34mm (threshold: 5.0mm)
[INFO] Filtering small motion: 3.12mm (threshold: 5.0mm)
# → No position update, good!

# Robot di chuyển
[INFO] Motion detected: 8.5cm
# → Position updates, good!
```

### ❌ SAI:
```
# Robot đứng yên
[INFO] Motion detected: 15.2cm  ← WRONG! Robot không động
[INFO] Motion detected: 22.7cm  ← Baseline sai!
```

---

## Câu Hỏi Thường Gặp

**Q: Tại sao baseline quan trọng đến vậy?**
A: Baseline sai → depth sai → SLAM tính motion sai. 
   VD: Baseline 60mm khi thật ra 140mm = depth sai 2.3× = motion estimate sai hoàn toàn!

**Q: Làm sao biết baseline thật của camera?**
A: Đo khoảng cách giữa center của 2 camera lenses. 
   Hoặc dùng `check_calibration.py` để xem calibrated baseline.

**Q: Có cần chạy lại calibration không?**
A: Nếu RMS error > 1.0 hoặc baseline không match hardware → YES, recalibrate!
   Nếu RMS < 0.5 và baseline đúng → NO, chỉ cần update config.

**Q: Motion filtering có làm chậm response không?**
A: Không. Nó chỉ filter noise (<5mm). Real motions (>5mm) vẫn detect ngay.

---

## Files Đã Update

- ✅ `main.py`: Added baseline config, logging, verification
- ✅ `stereo_camera.py`: Baseline parameter support, default 140mm
- ✅ `orb_slam3_wrapper.py`: Motion filtering (5mm threshold, 50cm sanity check)
- ✅ `check_calibration.py`: Tool để verify calibration và baseline

---

## Monitoring

### Real-time monitoring:
```bash
# Terminal 1: Run robot
python3 main.py

# Terminal 2: Monitor logs
tail -f robot.log | grep -E "baseline|Motion|Filtering"

# Xem:
# - Baseline being used
# - Motion detections
# - Filtering operations
```

### Check SLAM pose:
```python
# Trong web dashboard hoặc code
pose = slam.get_2d_pose()
print(f"Position: x={pose[0]:.3f}m, y={pose[1]:.3f}m, θ={pose[2]:.3f}rad")

# Nếu robot đứng yên → pose không nên thay đổi
```

---

**Last Updated:** 2026-02-19  
**Hardware:** Stereo camera with 140mm baseline  
**SLAM:** SimpleVisualSLAM with motion filtering
