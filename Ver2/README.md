# Autonomous Mapping Robot — Ver2

A tracked mobile robot that autonomously explores and maps indoor environments using stereo vision, sensor fusion, and A\* path planning. All computation runs on a NVIDIA Jetson Nano.

---

## Table of Contents

- [Hardware Configuration](#hardware-configuration)
- [Software Architecture](#software-architecture)
- [Capabilities](#capabilities)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Getting Started](#getting-started)
- [Web Interface](#web-interface)
- [Known Limitations](#known-limitations)

---

## Hardware Configuration

### Compute Platform

| Component | Specification |
|---|---|
| **Board** | NVIDIA Jetson Nano |
| **OS interface** | Jetson.GPIO (direct header pin control) |
| **IMU bus** | I2C Bus 1 |

### Drive System

| Component | Specification |
|---|---|
| **Type** | Differential tracked drive (tank-style) |
| **Motors** | 2× stepper motors |
| **Drivers** | 2× A4988 microstepping drivers |
| **Microstepping** | 1/16 step (configurable: full, 1/2, 1/4, 1/8, 1/16) |
| **Steps per revolution** | 200 (full step) × 16 = 3200 steps/rev |
| **Sprocket outer diameter** | 69.84 mm (from CAD) |
| **Track separation (wheel base)** | 284.5 mm (centre-to-centre) |
| **Gear ratio** | 1:1 (direct drive — motor shaft connected directly to 20-tooth sprocket) |
| **Max linear speed** | 0.3 m/s |
| **Max angular speed** | 1.5 rad/s |

#### Motor GPIO Pinout (Jetson Nano Board Header)

| Signal | Left Motor Pin | Right Motor Pin |
|---|---|---|
| STEP | 33 | 32 |
| DIR | 35 | 36 |
| ENABLE | 37 | 38 |
| MS1 | 29 | 13 |
| MS2 | 31 | 15 |
| MS3 | 11 | 16 |

### Stereo Camera

| Component | Specification |
|---|---|
| **Type** | Dual USB cameras |
| **Left camera ID** | 1 |
| **Right camera ID** | 0 |
| **Resolution** | 640 × 480 pixels |
| **Frame rate** | 30 fps |
| **Stereo baseline** | 140 mm |
| **Calibration file** | `data/calibration.npz` |
| **Exposure control** | Manual (exposure: −5, gain: 55) — auto-exposure disabled |

### IMU — MPU-6050

| Component | Specification |
|---|---|
| **Chip** | InvenSense MPU-6050 |
| **Interface** | I2C, address 0x68, bus 1 |
| **Gyroscope range** | ±250 °/s |
| **Accelerometer range** | ±2 g |
| **Calibration** | Static bias calibration on startup (500 samples, robot must be stationary) |

---

## Software Architecture

```
main.py  (RobotSystem orchestrator + watchdog)
│
├── stereo_camera.py        — Frame capture, rectification, disparity, visual odometry
├── mpu6050_imu.py          — Raw IMU driver + complementary filter
├── imu_fusion.py           — Extended Kalman Filter, FusedSLAMWrapper
├── orb_slam3_wrapper.py    — ORB-SLAM3 bindings (optional) + SimpleVisualSLAM fallback
├── obstacle_detector.py    — Depth-map obstacle detection across 5 spatial zones
├── autonomous_mapper.py    — OccupancyGrid + AutonomousMapper (A* + frontier exploration)
├── jetson_motor_controller.py — A4988 GPIO stepper control + odometry
├── safety_controller.py    — SafetyVelocityController (obstacle-aware teleoperation)
└── web_server.py           — Flask REST API + live dashboard (port 5000)
```

### Module Descriptions

#### `stereo_camera.py` — Stereo Camera & Visual Odometry
Manages two USB cameras in a background capture thread. Loads stereo calibration (intrinsics, distortion, rotation, translation) from `data/calibration.npz`, rectifies frames, and computes a dense disparity map using OpenCV's `StereoBM`/`StereoSGBM`. The `VisualOdometry` class estimates incremental pose changes by matching ORB features between consecutive rectified frames and recovering the essential matrix.

#### `mpu6050_imu.py` — IMU Driver
Full register-level I2C driver for the MPU-6050. Runs a continuous background reading thread. Applies static bias calibration at startup. A `ComplementaryFilter` class integrates gyroscope and accelerometer to produce stable roll/pitch/yaw estimates.

#### `imu_fusion.py` — Sensor Fusion (EKF)
An `ExtendedKalmanFilter` maintains state **[x, y, θ, vx, vy, ω]**. The IMU gyroscope drives the prediction step; visual odometry pose updates from SLAM drive the correction step. `FusedSLAMWrapper` wraps any SLAM object with this EKF so it is a transparent drop-in replacement. Configurable IMU weight (default 0.3).

#### `orb_slam3_wrapper.py` — SLAM
- **ORB-SLAM3** (optional): requires compiled C++ Python bindings in `/home/jetson/ORB_SLAM3`. Disabled by default (`use_orbslam3: false` in config).
- **SimpleVisualSLAM** (default): feature-based stereo VO using ORB descriptors. Estimates 3-D motion from matched keypoints and stereo depth. Suitable for embedded deployment without external dependencies.

#### `obstacle_detector.py` — Obstacle Detection
Converts the stereo disparity map to a metric depth map then analyses five zones:

| Zone | Angle Range |
|---|---|
| `front_center` | 0° |
| `front_left` | −45° to 0° |
| `front_right` | 0° to +45° |
| `left` | −90° to −45° |
| `right` | +45° to +90° |

Maintains a rolling history of 5 frames to suppress noise. Outputs a recommended action (`clear`, `turn_left`, `turn_right`, `stop`) and raises emergency stop at critical distance (0.15 m).

#### `autonomous_mapper.py` — Mapping & Navigation
- **`OccupancyGrid`**: 10 m × 10 m grid at 5 cm/cell resolution (200 × 200 cells). Cells: `0` = free, `100` = occupied, `−1` = unknown.
- **`AutonomousMapper`**: implements frontier-based exploration (seeks boundaries between known-free and unknown cells), inflates obstacles by robot radius (0.2 m) before planning, and uses **A\*** to compute collision-free paths. Saves completed maps to `data/room_map.npz`.

#### `jetson_motor_controller.py` — Motor Control
Direct Jetson GPIO stepper control. Each `A4988StepperMotor` runs a dedicated background thread generating STEP pulses at the requested frequency. Tracks total signed step count for dead-reckoning odometry. `JetsonMotorController` coordinates both motors for differential-drive kinematics (`move_forward`, `rotate`, `set_velocity`). `SafetyController` enforces hard speed limits.

#### `safety_controller.py` — Safe Teleoperation
`SafetyVelocityController` intercepts manual velocity commands, checks the latest obstacle-detector result, and either passes, scales, or blocks the command before it reaches the motors — ensuring the robot cannot be driven into an obstacle during remote operation.

#### `web_server.py` — Web Dashboard (Flask, port 5000)
Provides:
- **`GET /`** — main dashboard (live camera + map)
- **`GET /api/status`** — JSON: motor stats, SLAM stats, mapper stats, IMU availability, EKF fusion stats
- **`GET /api/pose`** — current 2-D pose (x, y, θ)
- **`GET /api/map`** — occupancy grid as base64 PNG
- **`POST /api/command`** — send velocity or navigation commands
- Live MJPEG camera stream

---

## Capabilities

| Capability | Details |
|---|---|
| **Autonomous room mapping** | Frontier-based exploration of unknown spaces; builds and saves a 2-D occupancy grid |
| **Visual SLAM** | Stereo feature-based pose estimation; optional upgrade to full ORB-SLAM3 |
| **IMU–Visual sensor fusion** | EKF fuses gyroscope data with visual odometry to reduce drift |
| **Real-time obstacle avoidance** | Depth map analysis; stops or steers before collision (critical threshold 0.15 m) |
| **A\* path planning** | Computes shortest collision-free path on the occupancy grid with obstacle inflation |
| **Odometry** | Step-count dead-reckoning combined with visual + IMU estimates |
| **Remote monitoring** | Live video stream, telemetry, and SLAM map via browser at `http://<robot-ip>:5000` |
| **Remote manual control** | Teleoperation through the web dashboard with obstacle-safety enforcement |
| **Watchdog health monitoring** | Background thread detects camera freeze and triggers safe shutdown |
| **Config-driven hardware** | All pin assignments, dimensions, speed limits, and tuning in `config.yaml` |

---

## Project Structure

```
Ver2/
├── main.py                         # Entry point — RobotSystem orchestrator
├── config.yaml                     # All hardware & tuning parameters
├── autonomous_mapper.py            # OccupancyGrid, AutonomousMapper, A*
├── imu_fusion.py                   # Extended Kalman Filter, FusedSLAMWrapper
├── jetson_motor_controller.py      # A4988 GPIO stepper drivers, odometry
├── mpu6050_imu.py                  # MPU-6050 I2C driver, ComplementaryFilter
├── obstacle_detector.py            # Stereo depth obstacle detection
├── orb_slam3_wrapper.py            # ORB-SLAM3 bindings + SimpleVisualSLAM
├── safety_controller.py            # Obstacle-aware velocity gating
├── stereo_camera.py                # Stereo capture, rectification, disparity, VO
├── vision_debug_server.py          # Debug vision stream server
├── web_server.py                   # Flask dashboard & REST API
├── data/
│   ├── calibration.npz             # Stereo camera calibration data
│   └── room_map.npz                # Saved occupancy grid map
├── docs/                           # Hardware guides and documentation
├── logs/                           # Runtime log files
├── templates/
│   ├── index.html                  # Main web dashboard
│   └── vision_debug.html           # Vision debug view
├── tests/                          # Unit and integration tests
│   ├── imu/
│   ├── motors/
│   └── sensors/
└── tools/                          # Calibration and debug utilities
    ├── calibration/
    ├── camera/
    └── debug/
```

---

## Configuration

All parameters are in [`config.yaml`](config.yaml). No Python source changes are needed for hardware tuning.

Key sections:

```yaml
camera:
  stereo_baseline: 0.14       # metres — must match physical hardware
  width: 640
  height: 480
  fps: 30

motor:
  wheel_diameter: 0.06984     # metres (69.84 mm outer sprocket diameter)
  wheel_base: 0.2845          # metres (284.5 mm track separation)
  gear_ratio: 1.0             # direct drive (1:1, no intermediate gears)
  microsteps: 16
  max_linear_speed: 0.3       # m/s
  max_angular_speed: 1.5      # rad/s

mapping:
  grid_width: 10.0            # metres
  grid_height: 10.0           # metres
  resolution: 0.05            # metres per cell

obstacle_detection:
  min_safe_distance: 0.3      # metres
  critical_distance: 0.15     # metres

imu:
  use_ekf: true
  fusion_weight: 0.3          # IMU contribution to fused pose

slam:
  use_orbslam3: false         # set true only when C++ bindings are built
```

---

## Getting Started

### Dependencies

```bash
pip install flask opencv-python numpy smbus2 pyyaml
# Jetson Nano only:
pip install Jetson.GPIO
```

### Camera Calibration

Before the first run, calibrate the stereo camera pair using the provided tools:

```bash
python tools/calibration/calibrate_motors.py
```

Calibration data is saved to `data/calibration.npz`.

### Run

```bash
python main.py
```

The system initialises all components in order (camera → IMU → SLAM → motors → obstacle detector → occupancy grid → mapper), then starts the web server and main processing loop.

> **Note:** Keep the robot **stationary** for approximately 2–3 seconds at startup while the IMU performs its bias calibration (500 samples).

### Web Dashboard

Open a browser and navigate to:

```
http://<jetson-ip>:5000
```

---

## Web Interface

The dashboard provides:

- **Live camera feed** — left and right stereo frames
- **SLAM map view** — real-time occupancy grid as the robot explores
- **Telemetry panel** — pose (x, y, θ), velocity, step counts, IMU readings, EKF state
- **Manual control** — arrow-key or on-screen joystick teleoperation (safety-gated)
- **Mapping controls** — start/stop autonomous exploration, save map

---

## Known Limitations

- **No LiDAR or ultrasonic sensors** — all depth perception is stereo-vision-only. Performance degrades in textureless or low-light environments.
- **ORB-SLAM3 disabled by default** — requires separately compiled C++ Python bindings (`use_orbslam3: false` in config). The built-in `SimpleVisualSLAM` is less robust under fast motion or large rotations.
- **EKF uses gyroscope Z-axis only** — accelerometer-based translation correction is not fused; linear position still drifts over long distances.
- **Outdoor / multi-floor use not supported** — the occupancy grid is 2-D and flat; no elevation or multi-level mapping.
