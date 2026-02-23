#!/usr/bin/env python3
"""
Test đơn giản cho A4988 + NEMA 17
Chạy motor 1 vòng từ từ để kiểm tra kết nối
"""
import Jetson.GPIO as GPIO
import time
import sys

# ========== CẤU HÌNH ==========
# Thay đổi theo kết nối thực tế của bạn
STEP_PIN = 33       # Pin vật lý 33 (GPIO 13)
DIR_PIN = 35        # Pin vật lý 35 (GPIO 19)
ENABLE_PIN = 37     # Pin vật lý 37 (GPIO 26)

# MS pins (nếu muốn điều khiển qua GPIO)
# Để None nếu đấu cứng MS pins vào GND hoặc VDD
MS1_PIN = None      # Ví dụ: 11
MS2_PIN = None      # Ví dụ: 13
MS3_PIN = None      # Ví dụ: 15

# Microstepping setting (phải khớp với hardware!)
# 1 = Full step (MS pins → GND)
# 16 = 1/16 step (MS pins → VDD hoặc H,H,H)
MICROSTEPS = 16

# Tốc độ test (giây giữa mỗi bước)
STEP_DELAY = 0.002  # 2ms = 250 bước/giây
# ===============================


def setup_gpio():
    """Khởi tạo GPIO pins"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    # Setup main pins
    GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = disabled
    
    # Setup MS pins if configured
    if MS1_PIN is not None:
        GPIO.setup(MS1_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MS2_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MS3_PIN, GPIO.OUT, initial=GPIO.LOW)
        
        # Configure microstepping
        ms_config = {
            1:  (0, 0, 0),
            2:  (1, 0, 0),
            4:  (0, 1, 0),
            8:  (1, 1, 0),
            16: (1, 1, 1),
        }
        
        if MICROSTEPS in ms_config:
            ms1, ms2, ms3 = ms_config[MICROSTEPS]
            GPIO.output(MS1_PIN, GPIO.HIGH if ms1 else GPIO.LOW)
            GPIO.output(MS2_PIN, GPIO.HIGH if ms2 else GPIO.LOW)
            GPIO.output(MS3_PIN, GPIO.HIGH if ms3 else GPIO.LOW)
            print(f"✓ Microstepping set to 1/{MICROSTEPS} via GPIO")
        else:
            print(f"✗ Invalid microstep value: {MICROSTEPS}")
            return False
    else:
        print(f"⚠ MS pins not controlled by GPIO")
        print(f"  Make sure hardware MS pins match MICROSTEPS={MICROSTEPS}")
        print(f"  Full step (1): MS→GND | 1/16 step (16): MS→VDD")
    
    return True


def test_motor():
    """Test motor rotation"""
    print("=" * 60)
    print("A4988 + NEMA 17 MOTOR TEST")
    print("=" * 60)
    print(f"STEP Pin: {STEP_PIN}")
    print(f"DIR Pin: {DIR_PIN}")
    print(f"ENABLE Pin: {ENABLE_PIN}")
    print(f"Microstepping: 1/{MICROSTEPS}")
    print("=" * 60)
    
    if not setup_gpio():
        print("GPIO setup failed!")
        return
    
    try:
        # Enable motor driver
        print("\n[1] Enabling motor driver...")
        GPIO.output(ENABLE_PIN, GPIO.LOW)  # LOW = enabled
        time.sleep(0.5)
        print("✓ Motor enabled (you should hear a click/lock)")
        
        # Calculate steps for one full rotation
        base_steps = 200  # NEMA 17 standard
        total_steps = base_steps * MICROSTEPS
        
        print(f"\n[2] Rotating motor 1 full revolution ({total_steps} steps)...")
        print("    Direction: FORWARD")
        
        # Set direction forward
        GPIO.output(DIR_PIN, GPIO.HIGH)
        time.sleep(0.01)
        
        # Rotate forward
        start_time = time.time()
        for step in range(total_steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEP_DELAY / 2)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_DELAY / 2)
            
            # Progress indicator
            if (step + 1) % (total_steps // 10) == 0:
                progress = int(((step + 1) / total_steps) * 100)
                print(f"    Progress: {progress}% ({step + 1}/{total_steps} steps)")
        
        elapsed = time.time() - start_time
        rpm = (60.0 / elapsed)
        print(f"\n✓ Forward rotation complete!")
        print(f"  Time: {elapsed:.2f}s")
        print(f"  Speed: {rpm:.1f} RPM")
        
        time.sleep(1.0)
        
        # Rotate backward
        print(f"\n[3] Rotating motor 1 full revolution BACKWARD...")
        GPIO.output(DIR_PIN, GPIO.LOW)
        time.sleep(0.01)
        
        start_time = time.time()
        for step in range(total_steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEP_DELAY / 2)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_DELAY / 2)
            
            if (step + 1) % (total_steps // 10) == 0:
                progress = int(((step + 1) / total_steps) * 100)
                print(f"    Progress: {progress}% ({step + 1}/{total_steps} steps)")
        
        elapsed = time.time() - start_time
        rpm = (60.0 / elapsed)
        print(f"\n✓ Backward rotation complete!")
        print(f"  Time: {elapsed:.2f}s")
        print(f"  Speed: {rpm:.1f} RPM")
        
        # Continuous rotation test
        print(f"\n[4] Continuous rotation test (5 seconds)...")
        print("    Press Ctrl+C to stop early")
        
        GPIO.output(DIR_PIN, GPIO.HIGH)
        test_duration = 5.0
        start_time = time.time()
        step_count = 0
        
        while (time.time() - start_time) < test_duration:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEP_DELAY / 2)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_DELAY / 2)
            step_count += 1
        
        elapsed = time.time() - start_time
        revolutions = step_count / total_steps
        rpm = (revolutions / elapsed) * 60
        
        print(f"\n✓ Continuous test complete!")
        print(f"  Steps: {step_count}")
        print(f"  Revolutions: {revolutions:.2f}")
        print(f"  Average RPM: {rpm:.1f}")
        
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\n[5] Cleaning up...")
        GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable motor
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ GPIO cleaned up")
        print("\n" + "=" * 60)
        print("TEST COMPLETE")
        print("=" * 60)


def diagnose_issues():
    """Print diagnostic checklist"""
    print("\n" + "=" * 60)
    print("DIAGNOSTIC CHECKLIST")
    print("=" * 60)
    print("\nIf motor did NOT rotate properly, check:")
    print("[ ] 1. VREF voltage set correctly (0.4-0.5V for 1.5A motor)")
    print("[ ] 2. 12V power connected to A4988 VMOT")
    print("[ ] 3. GND common between Jetson and A4988")
    print("[ ] 4. All 4 motor wires connected to 1A, 1B, 2A, 2B")
    print("[ ] 5. MS pins configured correctly (not floating!)")
    print("[ ] 6. MICROSTEPS setting matches hardware configuration")
    print("[ ] 7. Motor coils paired correctly")
    print("\nIf motor just vibrated/buzzed:")
    print("  → MICROSTEPS mismatch! Check MS pin connections")
    print("  → Try swapping two wires of the same coil")
    print("\nIf motor was too weak:")
    print("  → Increase VREF (measure and adjust potentiometer)")
    print("\nIf motor skipped steps:")
    print("  → Decrease speed (increase STEP_DELAY)")
    print("  → Increase VREF current")
    print("=" * 60)


if __name__ == "__main__":
    print("\n*** A4988 + NEMA 17 Stepper Motor Test ***\n")
    print("This will test your motor configuration.")
    print("The motor should rotate smoothly 1 revolution forward,")
    print("then 1 revolution backward, then run for 5 seconds.\n")
    
    input("Press ENTER to start test (or Ctrl+C to abort)...")
    
    test_motor()
    diagnose_issues()
    
    print("\nRead A4988_DIAGNOSTIC.md for detailed troubleshooting!\n")
