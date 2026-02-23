#!/usr/bin/env python3
"""
Camera Manual Settings Tuner
Helps find optimal manual exposure and gain settings for stereo cameras
"""
import cv2
import numpy as np
import sys
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def test_camera_settings():
    """Interactive tool to find optimal camera settings"""
    logger.info("=" * 70)
    logger.info("Stereo Camera Manual Settings Tuner")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This tool helps you find the best manual exposure and gain settings")
    logger.info("to avoid color mismatch between left and right cameras.")
    logger.info("")
    
    # Camera IDs
    left_id = int(input("Enter LEFT camera ID (default 1): ") or "1")
    right_id = int(input("Enter RIGHT camera ID (default 0): ") or "0")
    
    # Open cameras
    logger.info(f"\nOpening cameras {left_id} and {right_id}...")
    left_cam = cv2.VideoCapture(left_id)
    right_cam = cv2.VideoCapture(right_id)
    
    if not left_cam.isOpened() or not right_cam.isOpened():
        logger.error("Failed to open cameras!")
        return 1
    
    # Set resolution
    for cam in [left_cam, right_cam]:
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cam.set(cv2.CAP_PROP_FPS, 30)
    
    # Manual settings
    exposure = -6
    gain = 50
    wb_temp = 4600
    
    logger.info("\n" + "=" * 70)
    logger.info("KEYBOARD CONTROLS:")
    logger.info("=" * 70)
    logger.info("  Exposure:")
    logger.info("    [q] Darker (decrease exposure)")
    logger.info("    [w] Brighter (increase exposure)")
    logger.info("  Gain:")
    logger.info("    [a] Less gain (less noise)")
    logger.info("    [s] More gain (brighter but noisier)")
    logger.info("  White Balance:")
    logger.info("    [z] Cooler (decrease temperature)")
    logger.info("    [x] Warmer (increase temperature)")
    logger.info("  Other:")
    logger.info("    [r] Reset to defaults")
    logger.info("    [SPACE] Save current settings")
    logger.info("    [ESC] Exit")
    logger.info("=" * 70)
    
    def apply_settings():
        """Apply current settings to both cameras"""
        for cam, name in [(left_cam, "LEFT"), (right_cam, "RIGHT")]:
            # Set manual mode
            cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cam.set(cv2.CAP_PROP_AUTO_WB, 0)
            cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            
            # Set values
            cam.set(cv2.CAP_PROP_EXPOSURE, exposure)
            cam.set(cv2.CAP_PROP_GAIN, gain)
            cam.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp)
            cam.set(cv2.CAP_PROP_BRIGHTNESS, 128)
            cam.set(cv2.CAP_PROP_FOCUS, 0)
    
    # Apply initial settings
    apply_settings()
    
    # Wait for cameras to adjust
    logger.info("\nWaiting for cameras to adjust...")
    for _ in range(30):
        left_cam.read()
        right_cam.read()
    
    logger.info("Ready! Adjust settings using keyboard controls.")
    logger.info("Watch for color consistency between left and right images.\n")
    
    while True:
        ret_l, left = left_cam.read()
        ret_r, right = right_cam.read()
        
        if not ret_l or not ret_r:
            logger.error("Failed to capture frames!")
            break
        
        # Create side-by-side view
        combined = np.hstack([left, right])
        
        # Add line in the middle to see alignment
        h, w = combined.shape[:2]
        cv2.line(combined, (w//2, 0), (w//2, h), (0, 255, 0), 2)
        
        # Add settings overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_offset = 30
        cv2.putText(combined, f"Exposure: {exposure}", (10, y_offset), 
                   font, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, f"Gain: {gain}", (10, y_offset + 30), 
                   font, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, f"WB Temp: {wb_temp}K", (10, y_offset + 60), 
                   font, 0.7, (0, 255, 0), 2)
        
        # Calculate brightness difference between left and right
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        left_mean = left_gray.mean()
        right_mean = right_gray.mean()
        brightness_diff = abs(left_mean - right_mean)
        
        # Show brightness difference
        color = (0, 255, 0) if brightness_diff < 10 else (0, 165, 255) if brightness_diff < 20 else (0, 0, 255)
        cv2.putText(combined, f"Brightness diff: {brightness_diff:.1f}", 
                   (10, y_offset + 90), font, 0.7, color, 2)
        
        if brightness_diff < 10:
            cv2.putText(combined, "GOOD MATCH!", (10, y_offset + 120), 
                       font, 0.7, (0, 255, 0), 2)
        elif brightness_diff < 20:
            cv2.putText(combined, "OK", (10, y_offset + 120), 
                       font, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(combined, "POOR MATCH - Adjust!", (10, y_offset + 120), 
                       font, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("Stereo Camera Tuner (LEFT | RIGHT)", combined)
        
        key = cv2.waitKey(1) & 0xFF
        
        changed = False
        
        if key == ord('q'):  # Darker
            exposure = max(-13, exposure - 1)
            changed = True
            logger.info(f"Exposure: {exposure}")
        elif key == ord('w'):  # Brighter
            exposure = min(-1, exposure + 1)
            changed = True
            logger.info(f"Exposure: {exposure}")
        elif key == ord('a'):  # Less gain
            gain = max(0, gain - 10)
            changed = True
            logger.info(f"Gain: {gain}")
        elif key == ord('s'):  # More gain
            gain = min(100, gain + 10)
            changed = True
            logger.info(f"Gain: {gain}")
        elif key == ord('z'):  # Cooler WB
            wb_temp = max(2800, wb_temp - 200)
            changed = True
            logger.info(f"WB Temperature: {wb_temp}K")
        elif key == ord('x'):  # Warmer WB
            wb_temp = min(6500, wb_temp + 200)
            changed = True
            logger.info(f"WB Temperature: {wb_temp}K")
        elif key == ord('r'):  # Reset
            exposure = -6
            gain = 50
            wb_temp = 4600
            changed = True
            logger.info("Reset to defaults")
        elif key == ord(' '):  # Save
            logger.info("\n" + "=" * 70)
            logger.info("RECOMMENDED SETTINGS FOR main.py config:")
            logger.info("=" * 70)
            logger.info(f"'manual_exposure': {exposure},")
            logger.info(f"'manual_gain': {gain},")
            logger.info(f"# WB Temperature: {wb_temp}K")
            logger.info("=" * 70)
            logger.info("")
        elif key == 27:  # ESC
            break
        
        if changed:
            apply_settings()
            # Flush a few frames
            for _ in range(5):
                left_cam.read()
                right_cam.read()
    
    # Cleanup
    left_cam.release()
    right_cam.release()
    cv2.destroyAllWindows()
    
    logger.info("\n" + "=" * 70)
    logger.info("FINAL SETTINGS:")
    logger.info("=" * 70)
    logger.info(f"'manual_exposure': {exposure},")
    logger.info(f"'manual_gain': {gain},")
    logger.info(f"# WB Temperature: {wb_temp}K (not configurable in current version)")
    logger.info("=" * 70)
    logger.info("\nUpdate these values in main.py config['camera'] section.")
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(test_camera_settings())
    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}")
        sys.exit(1)
