#!/usr/bin/env python3
"""
Verify Stereo Camera Calibration và Baseline
Kiểm tra và hiển thị thông tin calibration
"""
import numpy as np
import sys
import os

def check_calibration(calibration_file='calibration.npz'):
    """Check calibration file and display parameters"""
    print("=" * 70)
    print("STEREO CAMERA CALIBRATION CHECKER")
    print("=" * 70)
    print()
    
    if not os.path.exists(calibration_file):
        print(f"❌ Calibration file NOT found: {calibration_file}")
        print()
        print("Hệ thống sẽ dùng DEFAULT calibration:")
        print("  - Baseline: 140mm (default)")
        print("  - Focal length: 500px (ước tính)")
        print("  - No distortion correction")
        print()
        print("⚠️ WARNING: Depth measurements sẽ KHÔNG CHÍNH XÁC!")
        print()
        print("Để tạo calibration file:")
        print("  1. Chạy: python3 calibrate_stereo.py")
        print("  2. Chụp 20-30 ảnh chessboard từ nhiều góc")
        print("  3. Calibration sẽ lưu vào calibration.npz")
        return False
    
    try:
        # Load calibration
        print(f"✓ Found calibration file: {calibration_file}")
        data = np.load(calibration_file)
        
        print("\n" + "=" * 70)
        print("CALIBRATION PARAMETERS")
        print("=" * 70)
        
        # Camera matrices
        if 'K_left' in data:
            K_left = data['K_left']
            print("\n[LEFT CAMERA MATRIX]")
            print(K_left)
            fx_left = K_left[0, 0]
            fy_left = K_left[1, 1]
            cx_left = K_left[0, 2]
            cy_left = K_left[1, 2]
            print(f"  Focal length (fx, fy): ({fx_left:.2f}, {fy_left:.2f}) pixels")
            print(f"  Principal point (cx, cy): ({cx_left:.2f}, {cy_left:.2f}) pixels")
        
        if 'K_right' in data:
            K_right = data['K_right']
            print("\n[RIGHT CAMERA MATRIX]")
            print(K_right)
            fx_right = K_right[0, 0]
            fy_right = K_right[1, 1]
            cx_right = K_right[0, 2]
            cy_right = K_right[1, 2]
            print(f"  Focal length (fx, fy): ({fx_right:.2f}, {fy_right:.2f}) pixels")
            print(f"  Principal point (cx, cy): ({cx_right:.2f}, {cy_right:.2f}) pixels")
        
        # Distortion
        if 'D_left' in data:
            D_left = data['D_left']
            print(f"\n[LEFT DISTORTION]: {D_left.flatten()}")
            distortion_magnitude = np.linalg.norm(D_left)
            if distortion_magnitude < 0.1:
                print("  ✓ Low distortion")
            else:
                print(f"  ⚠ Significant distortion: {distortion_magnitude:.3f}")
        
        if 'D_right' in data:
            D_right = data['D_right']
            print(f"[RIGHT DISTORTION]: {D_right.flatten()}")
        
        # Stereo parameters
        if 'T' in data:
            T = data['T']
            print(f"\n[TRANSLATION VECTOR (T)]")
            print(T)
            
            # Baseline is the X translation
            baseline = abs(T[0, 0]) if T.shape == (3, 1) else abs(T[0])
            print(f"\n⭐ STEREO BASELINE: {baseline*1000:.2f}mm ({baseline:.4f}m)")
            
            # Check if baseline is reasonable
            if baseline < 0.05:
                print("  ⚠️ WARNING: Baseline seems too small (<50mm)")
                print("     Expected range: 60-200mm for typical stereo setups")
            elif baseline > 0.30:
                print("  ⚠️ WARNING: Baseline seems too large (>300mm)")
                print("     Expected range: 60-200mm for typical stereo setups")
            else:
                print("  ✓ Baseline is in reasonable range")
            
            # Report hardware baseline
            print(f"\n📏 Hardware baseline (measured): 140mm")
            if abs(baseline - 0.14) > 0.01:
                print(f"  ⚠️ MISMATCH! Calibrated: {baseline*1000:.1f}mm vs Hardware: 140mm")
                print(f"     Difference: {abs(baseline - 0.14)*1000:.1f}mm")
                print("     → Depth measurements may be inaccurate!")
            else:
                print(f"  ✓ MATCH! Calibration matches hardware")
        
        if 'R' in data:
            R = data['R']
            print(f"\n[ROTATION MATRIX (R)]")
            print(R)
            
            # Check if cameras are aligned (R should be close to identity)
            R_identity_error = np.linalg.norm(R - np.eye(3))
            print(f"  Alignment error: {R_identity_error:.4f}")
            if R_identity_error < 0.1:
                print("  ✓ Cameras are well aligned")
            else:
                print("  ⚠ Cameras have significant misalignment")
        
        # Reprojection error (if available)
        if 'rms_error' in data:
            rms = float(data['rms_error'])
            print(f"\n[CALIBRATION QUALITY]")
            print(f"  RMS reprojection error: {rms:.3f} pixels")
            if rms < 0.5:
                print("  ✓ Excellent calibration!")
            elif rms < 1.0:
                print("  ✓ Good calibration")
            elif rms < 2.0:
                print("  ⚠ Fair calibration - consider recalibrating")
            else:
                print("  ❌ Poor calibration - MUST recalibrate!")
        
        print("\n" + "=" * 70)
        print("DEPTH ACCURACY ESTIMATION")
        print("=" * 70)
        
        if 'K_left' in data and 'T' in data:
            fx = K_left[0, 0]
            baseline_m = abs(T[0, 0]) if T.shape == (3, 1) else abs(T[0])
            
            print(f"\nDepth formula: Z = (f × B) / disparity")
            print(f"  f (focal length): {fx:.2f} pixels")
            print(f"  B (baseline): {baseline_m*1000:.2f}mm")
            print(f"  f × B = {fx * baseline_m:.2f} pixel·meters")
            
            print(f"\nExample depth calculations:")
            for disparity in [10, 20, 50, 100]:
                depth = (fx * baseline_m) / disparity
                print(f"  Disparity {disparity:3d} pixels → Depth: {depth:.2f}m ({depth*100:.0f}cm)")
            
            print(f"\nDepth range:")
            min_disparity = 10  # typical minimum
            max_disparity = 200  # typical maximum
            max_depth = (fx * baseline_m) / min_disparity
            min_depth = (fx * baseline_m) / max_disparity
            print(f"  Minimum depth: ~{min_depth:.2f}m ({min_depth*100:.0f}cm)")
            print(f"  Maximum depth: ~{max_depth:.2f}m ({max_depth*100:.0f}cm)")
        
        print("\n" + "=" * 70)
        print("RECOMMENDATIONS")
        print("=" * 70)
        
        # Check if we have good calibration
        issues = []
        if 'rms_error' in data and float(data['rms_error']) > 1.0:
            issues.append("High reprojection error")
        
        if 'T' in data:
            baseline_m = abs(T[0, 0]) if T.shape == (3, 1) else abs(T[0])
            if abs(baseline_m - 0.14) > 0.01:
                issues.append("Baseline mismatch with hardware")
        
        if issues:
            print("\n⚠️ Issues found:")
            for issue in issues:
                print(f"  - {issue}")
            print("\n📋 Recommended actions:")
            print("  1. Re-run stereo calibration: python3 calibrate_stereo.py")
            print("  2. Verify camera mounting (baseline should be 140mm)")
            print("  3. Take calibration photos with good chessboard visibility")
            print("  4. Ensure cameras are rigidly mounted (no flex)")
        else:
            print("\n✓ Calibration looks good!")
            print("  - Baseline matches hardware")
            print("  - Low reprojection error")
            print("  - Ready for accurate depth measurements")
        
        print("\n" + "=" * 70)
        
        return True
        
    except Exception as e:
        print(f"\n❌ Error reading calibration file: {e}")
        import traceback
        traceback.print_exc()
        return False


def estimate_baseline_from_images():
    """Interactive tool to estimate baseline from test images"""
    print("\n" + "=" * 70)
    print("BASELINE ESTIMATION TOOL")
    print("=" * 70)
    print("\nThis tool helps estimate baseline from stereo images.")
    print("\nYou need:")
    print("  1. Synchronized left/right images")
    print("  2. A known object at known distance")
    print("  3. Measure disparity of that object")
    print("\nFormula: Baseline = (Disparity × Distance) / FocalLength")
    print()
    
    try:
        # User input
        focal_length = float(input("Enter focal length (pixels) [500]: ") or "500")
        disparity = float(input("Enter measured disparity (pixels): "))
        distance = float(input("Enter known distance to object (meters): "))
        
        # Calculate baseline
        baseline = (disparity * distance) / focal_length
        
        print(f"\n⭐ Estimated BASELINE: {baseline*1000:.2f}mm ({baseline:.4f}m)")
        print(f"\nVerify: does this match your hardware measurement?")
        print(f"  Expected (hardware): 140mm")
        print(f"  Calculated: {baseline*1000:.2f}mm")
        
        if abs(baseline - 0.14) < 0.01:
            print("  ✓ MATCH!")
        else:
            print(f"  ⚠ Difference: {abs(baseline - 0.14)*1000:.1f}mm")
        
    except ValueError:
        print("\n❌ Invalid input")
    except KeyboardInterrupt:
        print("\n\nCancelled")


if __name__ == "__main__":
    print("\n*** STEREO CAMERA CALIBRATION CHECKER ***\n")
    
    # Check for calibration file
    cal_file = 'calibration.npz'
    if len(sys.argv) > 1:
        cal_file = sys.argv[1]
    
    check_calibration(cal_file)
    
    # Offer baseline estimation
    print("\n" + "=" * 70)
    response = input("\nEstimate baseline from test images? [y/N]: ").strip().lower()
    if response == 'y':
        estimate_baseline_from_images()
    
    print("\n*** DONE ***\n")
