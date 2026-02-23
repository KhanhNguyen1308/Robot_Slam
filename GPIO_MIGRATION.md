# GPIO Motor Controller Migration Complete

## Summary
Successfully replaced RP2040 serial controller with **direct Jetson Nano GPIO control** using A4988 stepper motor drivers.

## What Changed

### New Files Created
1. **`jetson_motor_controller.py`** - GPIO-based motor controller for A4988 drivers
2. **`WIRING_GUIDE.md`** - Complete pin connections and setup instructions
3. **`test_gpio_motors.py`** - Test script to verify motor connections

### Modified Files
1. **`main_mapper.py`** - Updated to use JetsonMotorController instead of RP2040Controller
2. **`config.yaml`** - Replaced serial config with GPIO pin assignments

### Removed Dependencies
- No longer requires RP2040 microcontroller
- No USB serial communication needed
- All motor control runs directly on Jetson Nano

## Hardware Setup Required

### 1. Wire Connections (see WIRING_GUIDE.md for details)
```
Left Motor A4988:
- Pin 33 → STEP
- Pin 35 → DIR  
- Pin 37 → ENABLE

Right Motor A4988:
- Pin 32 → STEP
- Pin 36 → DIR
- Pin 38 → ENABLE

Common:
- Jetson GND → A4988 GND
- 12V supply → A4988 VMOT
```

### 2. A4988 Configuration
- Set microstepping: MS1, MS2, MS3 pins
- Adjust current limit potentiometer (VREF)
- Add heatsinks to drivers
- Use 100µF capacitor on motor power

## Testing

### Quick Motor Test
```bash
cd /home/jetson/Robot_Slam

# Test motors individually (robot should be on stand!)
sudo python3 test_gpio_motors.py
```

### Run Full System
```bash
# Run the full robot system
sudo python3 main_mapper.py
```

**Note:** GPIO requires sudo/root access on Jetson Nano

## Configuration

Edit `config.yaml` to customize:

```yaml
motor_controller:
  # Change pin numbers if needed
  left_step_pin: 33
  left_dir_pin: 35
  # ... etc
  
  # Adjust for your motors
  steps_per_rev: 200    # 1.8° steppers
  microsteps: 16        # Microstepping level

robot:
  # Calibrate for your robot
  wheel_diameter: 0.066  # meters
  wheel_base: 0.165      # meters
```

## Advantages of GPIO Control

✅ **Lower latency** - No serial communication overhead
✅ **Simpler hardware** - No external microcontroller needed
✅ **Direct control** - Precise timing with Jetson
✅ **Cost effective** - Fewer components
✅ **Easier debugging** - Everything on one system

## Safety Features

- Emergency stop functionality
- Velocity limiting (SafetyController)
- Automatic motor disable on shutdown
- GPIO cleanup on exit

## Troubleshooting

### Motors not moving
1. Check GPIO permissions: `sudo usermod -a -G gpio $USER`
2. Verify A4988 power supply
3. Check ENABLE pin (should be LOW when enabled)
4. Verify wiring with multimeter

### Motors stuttering
1. Check microstepping configuration
2. Adjust current limit (VREF)
3. Reduce max speed in config
4. Add capacitors to motor power

### Permission denied
- GPIO requires root: use `sudo` to run scripts
- Or add user to gpio group

## Next Steps

1. **Wire the motors** following WIRING_GUIDE.md
2. **Configure A4988 drivers** (microstepping, current limit)
3. **Test motors** with test_gpio_motors.py
4. **Calibrate robot** (wheel diameter, base width)
5. **Run main system** with main_mapper.py

## Reverting to RP2040 (if needed)

If you need to switch back:
```python
# In main_mapper.py, change import:
from serial_controller import RP2040Controller, SafetyController
# And update initialization code
```
