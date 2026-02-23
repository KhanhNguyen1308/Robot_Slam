#!/usr/bin/env python3
"""
SLAM Tracking Diagnostics Tool
Helps debug visual tracking issues by showing feature detection in real-time
"""
import cv2
import numpy as np
from stereo_camera import StereoCamera
from orbslam_interface import SimpleVisualOdometry
import time

def main():
    print("=" * 60)
    print("SLAM Tracking Diagnostics")
    print("=" * 60)
    print()
    
    # Initialize camera
    print("Initializing cameras...")
    camera = StereoCamera(
        left_id=0,
        right_id=1,
        calibration_file='calibration.npz',
        width=1280,
        height=720,
        fps=30
    )
    
    if not camera.open():
        print("❌ Failed to open cameras")
        return
    
    camera.start_capture()
    time.sleep(1)
    print("✓ Cameras initialized")
    print()
    
    # Initialize visual odometry
    print("Initializing visual odometry...")
    vo = SimpleVisualOdometry()
    print("✓ Visual odometry ready")
    print()
    
    print("Press 'q' to quit")
    print("Press 's' to save current frame")
    print("=" * 60)
    print()
    
    frame_count = 0
    
    try:
        while True:
            # Get frames
            left, right, timestamp = camera.get_frames()
            
            if left is None or right is None:
                continue
            
            frame_count += 1
            
            # Get rectified frames
            rect_left, rect_right = camera.rectify_frames(left, right)
            
            # Track
            pose = vo.track(rect_left, rect_right)
            
            # Enhance image for feature detection (same as VO does internally)
            if len(rect_left.shape) == 3:
                gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
            else:
                gray_left = rect_left
            enhanced = vo.clahe.apply(gray_left)
            
            # Detect features
            kp, des = vo.detector.detectAndCompute(enhanced, None)
            
            # Draw keypoints on color image
            if len(rect_left.shape) == 2:
                vis_img = cv2.cvtColor(rect_left, cv2.COLOR_GRAY2BGR)
            else:
                vis_img = rect_left.copy()
            
            vis_img = cv2.drawKeypoints(
                vis_img, 
                kp, 
                None, 
                color=(0, 255, 0),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )
            
            # Add status text
            status_color = (0, 255, 0) if pose is not None else (0, 0, 255)
            status_text = "TRACKING" if pose is not None else "LOST"
            
            cv2.putText(vis_img, f"Status: {status_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
            cv2.putText(vis_img, f"Features: {len(kp)}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            if hasattr(vo, 'tracking_quality'):
                quality_pct = int(vo.tracking_quality * 100)
                quality_color = (0, 255, 0) if quality_pct > 50 else (0, 165, 255) if quality_pct > 20 else (0, 0, 255)
                cv2.putText(vis_img, f"Quality: {quality_pct}%", (10, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, quality_color, 2)
            
            if hasattr(vo, 'consecutive_failures') and vo.consecutive_failures > 0:
                cv2.putText(vis_img, f"Failures: {vo.consecutive_failures}", (10, 150),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                y_offset = 190
            else:
                y_offset = 150
            
            cv2.putText(vis_img, f"Frame: {frame_count}", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Show motion history if available
            if hasattr(vo, 'motion_history') and len(vo.motion_history) > 0:
                avg_motion = np.mean(vo.motion_history, axis=0)
                cv2.putText(vis_img, f"Motion: dx={avg_motion[0]:.1f} dy={avg_motion[1]:.1f}", (10, y_offset + 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Show visualization
            cv2.imshow('Feature Detection', vis_img)
            
            # Show enhanced image
            cv2.imshow('Enhanced (CLAHE)', enhanced)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"debug_frame_{frame_count}.jpg"
                cv2.imwrite(filename, vis_img)
                print(f"💾 Saved: {filename}")
            
            # Print periodic stats
            if frame_count % 30 == 0:
                quality_info = ""
                if hasattr(vo, 'tracking_quality'):
                    quality_info = f", Quality={vo.tracking_quality:.2f}"
                failures_info = ""
                if hasattr(vo, 'consecutive_failures') and vo.consecutive_failures > 0:
                    failures_info = f", Failures={vo.consecutive_failures}"
                motion_info = ""
                if hasattr(vo, 'motion_history') and len(vo.motion_history) > 0:
                    avg_motion = np.mean(vo.motion_history, axis=0)
                    motion_info = f", Motion=[{avg_motion[0]:.1f}, {avg_motion[1]:.1f}]"
                print(f"Frame {frame_count}: Features={len(kp)}, Status={status_text}{quality_info}{failures_info}{motion_info}")
    
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    
    finally:
        print("\nCleaning up...")
        camera.stop_capture()
        camera.close()
        cv2.destroyAllWindows()
        print("✓ Done")

if __name__ == "__main__":
    main()
