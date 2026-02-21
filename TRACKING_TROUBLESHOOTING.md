# Tracking Lost - Solutions Guide

## Quick Fixes

### 1. Check Camera Exposure
The cameras might be too dark or overexposed:
```bash
# Check current settings
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_auto,exposure_absolute,brightness

# If too dark, increase brightness
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=48
v4l2-ctl -d /dev/video1 --set-ctrl=brightness=48
```

### 2. Improve Lighting
- Turn on room lights
- Avoid pointing camera at plain walls or blank surfaces
- Add visual features to environment (posters, patterns, objects)

### 3. Move Slower
- Visual tracking can fail with rapid motion
- Start with slow, smooth movements
- Avoid sudden turns or jerky movements

### 4. Check Environment
**Good for tracking:**
- Well-lit scenes
- Textured surfaces (posters, books, patterns)
- Medium-distance features (0.5m - 3m)
- Stable camera mounting

**Bad for tracking:**
- Plain white walls
- Very dark or very bright scenes
- Moving too fast
- Very close to surfaces (<30cm)
- Vibration/shaking

## Diagnostic Tools

### Run Tracking Diagnostics
```bash
cd /home/jetson/Robot_Slam
python3 diagnose_tracking.py
```

This will show:
- Real-time feature detection
- Number of features found
- Tracking quality
- Enhanced image (CLAHE applied)

Press 's' to save problem frames for analysis.

### Check Web API
```bash
curl http://localhost:5000/api/slam/diagnostics
```

Shows current tracking state and quality metrics.

## Improved Visual Odometry

The system now uses enhanced SimpleVisualOdometry with:

✅ **CLAHE** - Contrast enhancement for better feature detection
✅ **2000 features** - More robust tracking (was 500)
✅ **Ratio test** - Better feature matching  
✅ **RANSAC** - Outlier rejection
✅ **Lower thresholds** - Tracks with fewer features (5 minimum)
✅ **Motion validation** - Rejects unrealistic jumps

## Recommended Settings

### For Best Tracking
```yaml
# config.yaml
camera:
  width: 640      # Lower resolution = faster processing
  height: 480
  fps: 30
```

### Camera Settings (via v4l2-ctl)
```bash
# Optimal settings for visual tracking
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3        # Auto exposure
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=40          # Adjust 20-60
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=32            # Default
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=64          # Default
v4l2-ctl -d /dev/video0 --set-ctrl=sharpness=3            # Slight sharpening

# Repeat for video1 (right camera)
```

## Understanding the Logs

### Normal Operation
```
✓ Tracking recovered (was lost for 3 frames, quality: 0.75)
```
- Temporary loss is normal during fast motion or feature-poor scenes
- Quality 0.5-1.0 is good

### Problem Indicators
```
⚠ Tracking lost - insufficient features (frame 156)
⚠ Still lost tracking (42 frames)
```
- Persistent loss (>10 frames) indicates environment or lighting issues
- Check camera view and lighting

### Motion Warnings
```
⚠ Large motion detected: dx=127.3, dy=89.5 pixels
```
- Moving too fast for visual tracking
- Reduce speed or improve frame rate

## Common Issues

### Issue: "Tracking lost immediately on startup"
**Solution:**
1. Point camera at textured surface
2. Check camera is not blocked
3. Ensure adequate lighting
4. Wait a few frames for auto-exposure to adjust

### Issue: "Tracking works, then lost during movement"
**Solution:**
1. Move slower
2. Reduce max speed in config.yaml
3. Add more visual features to environment
4. Check camera mounting (reduce vibration)

### Issue: "Tracking never works"
**Solution:**
1. Run `diagnose_tracking.py` to see feature count
2. If features < 50, improve lighting or aim at textured surfaces
3. Check camera calibration is loaded
4. Verify cameras are rectifying properly

### Issue: "Tracking quality low even with many features"
**Solution:**
1. Features might be in bad locations (edges of image)
2. Moving too fast (motion blur)
3. Camera out of focus
4. Calibration might be incorrect

## Performance Impact

Visual odometry CPU usage:
- Feature detection: ~15-20% per camera
- Matching: ~5-10%
- **Total: ~40-50% CPU on Jetson Nano**

If CPU is maxed out:
1. Reduce camera resolution (640x480 instead of 1280x720)
2. Reduce camera FPS (15 instead of 30)
3. Reduce ORB features (1000 instead of 2000)

## Advanced: Tuning ORB Parameters

Edit `orbslam_interface.py` SimpleVisualOdometry.__init__():

```python
self.detector = cv2.ORB_create(
    nfeatures=2000,      # More = better tracking, higher CPU
    fastThreshold=10,    # Lower = more features, less distinctive
    edgeThreshold=15,    # Lower = features closer to edges
)
```

Adjust based on your needs:
- **High-speed navigation**: nfeatures=1000, fastThreshold=20
- **Precise mapping**: nfeatures=3000, fastThreshold=5
- **Low CPU**: nfeatures=500, reduce resolution

## Still Having Issues?

1. Check system logs: `tail -f logs/robot_system.log`
2. Run diagnostic tool: `python3 diagnose_tracking.py`
3. Test in well-lit area with clear features
4. Verify camera calibration: `python3 calibrate_stereo.py --verify`
5. Consider using ORB-SLAM3 for better tracking (if available)

## Tips for Good Tracking

🎯 **Environment:**
- Well-lit (avoid shadows)
- Textured surfaces (books, posters, furniture)
- Stable features (not moving)

🎯 **Motion:**
- Start slow (0.1 m/s linear)
- Smooth acceleration
- Gradual turns (<1 rad/s)

🎯 **Camera:**
- Firmly mounted (no wobble)
- Clean lenses
- Proper calibration
- Auto-exposure enabled
