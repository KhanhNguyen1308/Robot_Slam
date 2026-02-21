# Feature Tracking Debugging on Headless Jetson Nano

## Problem
The diagnostic tool `diagnose_tracking.py` requires X display and won't work on headless Jetson Nano.

## Solution: Web-Based Debugging

The web interface now includes a **Feature Tracking Debug View** that shows:
- Real-time feature detection visualization
- Number of features detected
- Tracking status (OK/LOST)
- Tracking quality percentage
- Frames lost count

### How to Use

1. **Start the robot system:**
```bash
cd /home/jetson/Robot_Slam
sudo python3 main_mapper.py
```

2. **Access web interface:**
Open a web browser on any device on the same network:
```
http://JETSON_IP:5000
```
Replace `JETSON_IP` with your Jetson Nano's IP address (check with `ifconfig` or `ip addr`)

3. **View Feature Tracking:**
Click the **"🔍 Feature Tracking"** button at the top of the page

4. **What You'll See:**
- Green dots = Detected ORB features
- Larger circles = Higher feature response (more distinctive)
- Status text overlay:
  - **Status**: OK (green) or LOST (red)
  - **Features**: Number of features detected
  - **Lost frames**: How many consecutive frames lost tracking
  - **Quality**: Tracking quality percentage (0-100%)

### Interpreting the Display

**Good Tracking:**
- 50+ features visible
- Status: OK (green)
- Quality: >50%
- Green dots distributed across image

**Poor/Lost Tracking:**
- <10 features visible
- Status: LOST (red)
- Quality: <30%
- Few or no green dots

**What to Check if Tracking is Poor:**
1. **Lighting** - Too dark or too bright
2. **Scene** - Plain walls have no features
3. **Motion** - Moving too fast causes blur
4. **Camera** - Blocked, dirty lens, or out of focus

### Available Video Views

Click buttons at top to switch views:
- **Normal View** - Left and right cameras side-by-side
- **Stereo View** - Combined stereo pair
- **🔍 Feature Tracking** - Shows detected ORB features
- **Obstacles** - Shows obstacle detection
- **Depth Map** - Shows depth estimation

### API Endpoint for Scripts

You can also query tracking status programmatically:

```bash
# Get tracking diagnostics
curl http://JETSON_IP:5000/api/slam/diagnostics

# Example response:
# {
#   "tracking_state": "OK",
#   "frames_lost": 0,
#   "tracking_quality": 0.75,
#   "frames_since_init": 156
# }
```

### Saving Debug Images

If you need to save frames for offline analysis:

```bash
# Take snapshot from web stream
wget http://JETSON_IP:5000/video/features -O debug_features.mjpeg

# Or use curl for single frame
curl http://JETSON_IP:5000/video/features --output debug_stream.mjpeg
```

### Remote Access

From another computer on the network:

**Linux/Mac:**
```bash
# SSH tunnel (if firewall blocks port 5000)
ssh -L 5000:localhost:5000 jetson@JETSON_IP

# Then access in browser:
http://localhost:5000
```

**Windows:**
Use PuTTY with port forwarding:
1. Connection > SSH > Tunnels
2. Source port: 5000
3. Destination: localhost:5000
4. Click "Add"
5. Connect and access http://localhost:5000

### Tips for Better Debugging

1. **Refresh the page** if video stream stops
2. **Check browser console** (F12) for errors
3. **Monitor system logs** in another terminal:
   ```bash
   journalctl -f | grep main_mapper
   ```
4. **Check system resources:**
   ```bash
   htop
   # GPU usage
   tegrastats
   ```

### Troubleshooting

**Video stream not loading:**
- Check firewall: `sudo ufw status`
- Verify server is running: `ps aux | grep main_mapper`
- Check port is listening: `netstat -tlnp | grep 5000`

**Feature view shows "No camera data":**
- Camera may not be initialized
- Check logs for camera errors
- Restart the system

**Poor performance:**
- Reduce camera resolution in config.yaml
- Lower stream FPS (currently 15fps)
- Close other applications

## Alternative: Screenshot via OpenCV

If you need to debug without the web interface:

```python
import cv2
from stereo_camera import StereoCamera
from orbslam_interface import SimpleVisualOdometry

camera = StereoCamera()
camera.open()
camera.start_capture()

vo = SimpleVisualOdometry()

# Get one frame
left, right, _ = camera.get_frames()
rect_left, rect_right = camera.rectify_frames(left, right)

# Detect features
gray = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
enhanced = vo.clahe.apply(gray)
kp, des = vo.detector.detectAndCompute(enhanced, None)

# Draw and save
vis = cv2.drawKeypoints(rect_left, kp, None, (0,255,0))
cv2.imwrite('debug_features.jpg', vis)
print(f"Saved debug_features.jpg with {len(kp)} features")

camera.close()
```

Then download the image:
```bash
scp jetson@JETSON_IP:~/Robot_Slam/debug_features.jpg .
```
