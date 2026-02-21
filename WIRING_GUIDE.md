# Jetson Nano GPIO to A4988 Wiring Guide

## Pin Connections (BOARD Numbering)

### Left Motor (A4988 Driver #1)
- **Pin 33 (GPIO 13)** → A4988 STEP
- **Pin 35 (GPIO 19)** → A4988 DIR
- **Pin 37 (GPIO 26)** → A4988 ENABLE
- **Pin 29 (GPIO 5)** → A4988 MS1 (optional - for software microstepping control)
- **Pin 31 (GPIO 6)** → A4988 MS2 (optional - for software microstepping control)
- **Pin 11 (GPIO 17)** → A4988 MS3 (optional - for software microstepping control)

### Right Motor (A4988 Driver #2)
- **Pin 32 (GPIO 12)** → A4988 STEP
- **Pin 36 (GPIO 16)** → A4988 DIR
- **Pin 38 (GPIO 20)** → A4988 ENABLE
- **Pin 13 (GPIO 27)** → A4988 MS1 (optional - for software microstepping control)
- **Pin 15 (GPIO 22)** → A4988 MS2 (optional - for software microstepping control)
- **Pin 16 (GPIO 23)** → A4988 MS3 (optional - for software microstepping control)

### Power Connections
- **Jetson GND** → Both A4988 GND pins
- **12V Power Supply +** → Both A4988 VMOT
- **12V Power Supply GND** → Common ground with Jetson

### A4988 to Stepper Motor
Each A4988 has 4 output pins (1A, 1B, 2A, 2B):
- Connect to your stepper motor coils
- Check motor datasheet for coil pairs

## A4988 Microstepping Configuration

### Option 1: Software Control (Recommended)
Connect MS1, MS2, MS3 pins to Jetson GPIO as shown above. The software will automatically configure microstepping based on `config.yaml`:

```yaml
motor_controller:
  microsteps: 16  # Can be changed to 1, 2, 4, 8, or 16
```

**Advantages:**
- Change microstepping without rewiring
- Dynamic adjustment possible
- Easy testing of different settings

### Option 2: Hardware Wiring (Fixed)
If not using software control, wire MS1, MS2, MS3 directly to power/ground:

| MS1 | MS2 | MS3 | Microstep Resolution |
|-----|-----|-----|---------------------|
| Low | Low | Low | Full step           |
| High| Low | Low | Half step           |
| Low | High| Low | Quarter step        |
| High| High| Low | Eighth step         |
| High| High| High| Sixteenth step      |

For 1/16 microstepping: Connect MS1, MS2, MS3 to VDD (3.3V)

## Power Supply Requirements
- **VMOT**: 8-35V (12V recommended for NEMA 17)
- **Current**: Depends on motors (typically 1-2A per motor)
- **Adjust current limit**: Turn potentiometer on A4988 (measure VREF)
  - VREF = Current Limit / 2.5
  - For 1A motors: VREF ≈ 0.4V

## Safety Notes
1. **Never disconnect/connect motors while powered**
2. **Add heatsinks to A4988 drivers**
3. **Set current limit before connecting motors**
4. **Use capacitors on motor power supply (100µF recommended)**

## Testing
```bash
# Test motor controller
python3 test_gpio_motors.py
```

## Pin Layout Reference
```
Jetson Nano 40-pin Header (BOARD numbering)
===========================================
     3.3V [ 1] [ 2] 5V
          [ 3] [ 4] 5V
          [ 5] [ 6] GND
          [ 7] [ 8]
      GND [ 9] [10]
 GPIO17  [11] [12]
 GPIO27  [13] [14] GND
 GPIO22  [15] [16] GPIO23 ← MS pins (right)
     3.3V[17] [18]
          [19] [20] GND
          [21] [22]
          [23] [24]
      GND [25] [26]
          [27] [28]
   GPIO5 [29] [30] GND     ← MS pins (left)
   GPIO6 [31] [32] GPIO12  ← STEP (right)
  GPIO13 [33] [34] GND     ← STEP (left)
  GPIO19 [35] [36] GPIO16  ← DIR (right)
  GPIO26 [37] [38] GPIO20  ← ENABLE (right)
      GND[39] [40]
      
Control pins per motor:
Left:  33,35,37 + 29,31,11 (MS)
Right: 32,36,38 + 13,15,16 (MS)
```
