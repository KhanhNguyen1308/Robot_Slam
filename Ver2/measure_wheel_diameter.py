#!/usr/bin/env python3
"""
Measure Actual Wheel Diameter from Calibration Test
Calculates effective wheel diameter based on actual movement
"""

import math

def calculate_wheel_diameter():
    """Calculate wheel diameter from calibration results"""
    print("=" * 70)
    print("WHEEL DIAMETER CALCULATOR")
    print("=" * 70)
    print()
    print("This tool calculates the actual wheel diameter based on")
    print("calibration test results.")
    print()
    
    # Get gear ratio
    print("Current gear ratio: 38/18 = 2.111")
    gear_ratio = 38.0 / 18.0
    
    # Get microstepping
    print("Microstepping: 1/16")
    microsteps = 16
    steps_per_rev = 200 * microsteps  # 3200
    
    # Get test results
    print()
    print("Enter calibration test results:")
    commanded_distance = float(input("  Commanded distance (meters): "))
    actual_distance = float(input("  Actual distance traveled (meters): "))
    
    print()
    print("=" * 70)
    print("CALCULATION")
    print("=" * 70)
    
    # Calculate what the diameter should be
    # Formula: steps_per_meter = (steps_per_rev × gear_ratio) / (π × diameter)
    # Rearranged: diameter = (steps_per_rev × gear_ratio) / (steps_per_meter × π)
    
    # From actual movement, calculate actual steps_per_meter
    # If robot moved actual_distance when commanded commanded_distance:
    # actual_steps_per_meter = commanded_steps / actual_distance
    # commanded_steps = commanded_distance × steps_per_meter_calculated
    
    # But we need to work backwards from the ratio
    ratio = actual_distance / commanded_distance
    
    # If we assumed a certain diameter and got ratio of actual movement:
    # actual_diameter = assumed_diameter × ratio
    
    # Let's ask what diameter they assumed
    print()
    assumed_diameter = float(input("What wheel diameter did you use in config (mm)? "))
    assumed_diameter_m = assumed_diameter / 1000.0
    
    # Calculate what the diameter should actually be
    actual_diameter_m = assumed_diameter_m * ratio
    actual_diameter_mm = actual_diameter_m * 1000.0
    
    print()
    print("RESULTS:")
    print(f"  Assumed diameter: {assumed_diameter:.2f}mm")
    print(f"  Movement ratio:   {ratio:.4f} ({ratio*100:.2f}%)")
    print(f"  Actual diameter:  {actual_diameter_mm:.2f}mm")
    
    # Calculate from sprocket geometry if provided
    print()
    print("=" * 70)
    print("FROM CAD DRAWING (if available)")
    print("=" * 70)
    
    outer_radius = input("Outer radius from CAD (mm, or press ENTER to skip): ").strip()
    if outer_radius:
        outer_r = float(outer_radius)
        outer_d = outer_r * 2
        print(f"  Outer diameter: {outer_d:.2f}mm")
        
        inner_radius = input("Inner radius from CAD (mm): ").strip()
        if inner_radius:
            inner_r = float(inner_radius)
            inner_d = inner_r * 2
            print(f"  Inner diameter: {inner_d:.2f}mm")
            
            pitch_d = (outer_d + inner_d) / 2
            print(f"  Average (pitch) diameter: {pitch_d:.2f}mm")
    
    print()
    print("=" * 70)
    print("RECOMMENDATION")
    print("=" * 70)
    print()
    print("Update main.py config with:")
    print(f"  'wheel_diameter': {actual_diameter_m:.4f},  # {actual_diameter_mm:.2f}mm")
    print()
    print("To verify, you can also:")
    print("1. Measure sprocket circumference with string: C = π × D")
    print("2. Count motor rotations for 1 full sprocket rotation")
    print("3. Use the calibrated value from this calculation")
    print()
    
    # Show steps per meter with new diameter
    circumference = math.pi * actual_diameter_m
    steps_per_meter = (steps_per_rev * gear_ratio) / circumference
    print(f"With this diameter:")
    print(f"  Circumference: {circumference*1000:.2f}mm")
    print(f"  Steps per meter: {steps_per_meter:.1f}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        calculate_wheel_diameter()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
