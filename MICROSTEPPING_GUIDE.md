# Microstepping Control Guide

## Overview
MS (MicroStep) pins on the A4988 driver control the step resolution. The motor controller now supports software control of these pins for dynamic microstepping adjustment.

## Pin Assignments (from config.yaml)

### Left Motor MS Pins
- MS1: Pin 29 (GPIO 5)
- MS2: Pin 31 (GPIO 6)
- MS3: Pin 11 (GPIO 17)

### Right Motor MS Pins
- MS1: Pin 13 (GPIO 27)
- MS2: Pin 15 (GPIO 22)
- MS3: Pin 16 (GPIO 23)

## Microstepping Modes

| Mode | Steps | Smoothness | Torque | Speed | Use Case |
|------|-------|------------|--------|-------|----------|
| 1/1  | 200   | ★☆☆☆☆     | ★★★★★ | ★★★★★| High torque, high speed |
| 1/2  | 400   | ★★☆☆☆     | ★★★★☆ | ★★★★☆| Balanced |
| 1/4  | 800   | ★★★☆☆     | ★★★☆☆ | ★★★☆☆| Medium precision |
| 1/8  | 1600  | ★★★★☆     | ★★☆☆☆ | ★★☆☆☆| High precision |
| 1/16 | 3200  | ★★★★★     | ★☆☆☆☆ | ★☆☆☆☆| Ultra smooth, precise positioning |

## Configuration

### Using config.yaml (Static)
```yaml
motor_controller:
  microsteps: 16  # Set to 1, 2, 4, 8, or 16
```

### Dynamic Control (Python)
```python
from jetson_motor_controller import JetsonMotorController

controller = JetsonMotorController()
controller.connect()

# Change microstepping on the fly
controller.set_microstepping(8)   # Switch to 1/8 stepping
controller.send_velocity(0.2, 0)  # Move forward

controller.set_microstepping(16)  # Switch to 1/16 for precision
controller.send_velocity(0.1, 0)  # Slow precise movement
```

## Trade-offs

### Higher Microstepping (1/16, 1/8)
**Advantages:**
- Smoother motion
- Less vibration and noise
- Better for low speeds
- More precise positioning

**Disadvantages:**
- Lower torque at each step
- Slower maximum speed
- Higher CPU load (more steps to process)
- Less efficient

### Lower Microstepping (1/1, 1/2)
**Advantages:**
- Maximum torque
- Highest speeds possible
- More efficient
- Lower CPU load

**Disadvantages:**
- More vibration
- Noisier operation
- Less smooth at low speeds
- Lower positioning precision

## Recommended Settings

### For SLAM Mapping (Default)
```yaml
microsteps: 16  # Maximum smoothness for camera stability
```

### For High-Speed Navigation
```yaml
microsteps: 4   # Balance between speed and smoothness
```

### For Maximum Torque (Heavy loads)
```yaml
microsteps: 2   # Near-maximum torque while maintaining reasonable smoothness
```

### For Testing/Debugging
```yaml
microsteps: 1   # Easiest to observe and verify motor behavior
```

## Wiring Options

### Option 1: Software Control (Recommended)
Wire MS pins to Jetson GPIO as specified. Allows dynamic adjustment.

### Option 2: Hardware Fixed
Leave MS pins disconnected from Jetson and wire directly:
- **1/16 step**: MS1, MS2, MS3 → VDD (3.3V)
- **1/8 step**: MS1, MS2 → VDD, MS3 → GND
- **1/4 step**: MS2 → VDD, MS1, MS3 → GND
- **1/2 step**: MS1 → VDD, MS2, MS3 → GND
- **Full step**: MS1, MS2, MS3 → GND

**Note**: If using hardware fixed, set MS pins to `null` in config.yaml:
```yaml
left_ms1_pin: null
left_ms2_pin: null
left_ms3_pin: null
```

## Testing Different Modes
```bash
# Run test with different microstepping
sudo python3 test_gpio_motors.py

# Or manually test in Python:
python3
>>> from jetson_motor_controller import JetsonMotorController
>>> c = JetsonMotorController()
>>> c.connect()
>>> c.enable()
>>> # Try different modes
>>> c.set_microstepping(1)   # Full step - should see/hear difference
>>> c.send_velocity(0.1, 0)
>>> c.set_microstepping(16)  # Micro step - much smoother
>>> c.send_velocity(0.1, 0)
>>> c.disable()
```

## Impact on Performance

### CPU Usage
- 1/1 step: ~1-2% CPU per motor
- 1/16 step: ~5-10% CPU per motor

### Maximum Speed (approximate with 200 step motors, 66mm wheels)
- 1/1 step: ~1.5 m/s
- 1/16 step: ~0.3 m/s

### Positioning Accuracy (with 66mm wheels)
- 1/1 step: ~1.0 mm
- 1/16 step: ~0.06 mm

## Troubleshooting

### Motors stuttering at low speeds
→ Increase microstepping (use 1/16 instead of 1/4)

### Motors can't reach desired speed
→ Decrease microstepping (use 1/4 instead of 1/16)

### Motors skipping steps under load
→ Decrease microstepping for more torque

### Too much vibration/noise
→ Increase microstepping for smoother operation
