#!/usr/bin/env python3
"""
Stereo Camera Calibration Tool
Calibrates stereo camera setup and saves calibration to .npz file
"""
import cv2
import numpy as np
import glob
import argparse
import os


class StereoCalibration:
    """Stereo camera calibration using checkerboard pattern"""
    
    def __init__(self, 
                 checkerboard_size=(9, 6),
                 square_size=0.025,  # 25mm squares
                 left_camera_id=0,
                 right_camera_id=1):
        
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        self.left_camera_id = left_camera_id
        self.right_camera_id = right_camera_id
        
        # Prepare object points
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 
                                     0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Storage for calibration
        self.objpoints = []  # 3D points in real world space
        self.imgpoints_left = []  # 2D points in left image plane
        self.imgpoints_right = []  # 2D points in right image plane
        
        self.image_size = None
        
    def capture_calibration_images(self, num_images=20, output_dir='calib_images'):
        """
        Capture calibration image pairs
        
        Args:
            num_images: Number of image pairs to capture
            output_dir: Directory to save images
        """
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Open cameras
        cap_left = cv2.VideoCapture(self.left_camera_id)
        cap_right = cv2.VideoCapture(self.right_camera_id)
        
        # Set resolution
        for cap in [cap_left, cap_right]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        print(f"Capturing {num_images} calibration image pairs...")
        print("Press SPACE to capture, ESC to exit")
        
        count = 0
        
        while count < num_images:
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()
            
            if not ret_left or not ret_right:
                print("Failed to capture frames")
                continue
            
            # Find chessboard corners
            gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
            
            ret_left, corners_left = cv2.findChessboardCorners(
                gray_left, self.checkerboard_size, None
            )
            ret_right, corners_right = cv2.findChessboardCorners(
                gray_right, self.checkerboard_size, None
            )
            
            # Draw corners
            display_left = frame_left.copy()
            display_right = frame_right.copy()
            
            if ret_left:
                cv2.drawChessboardCorners(
                    display_left, self.checkerboard_size, corners_left, ret_left
                )
            if ret_right:
                cv2.drawChessboardCorners(
                    display_right, self.checkerboard_size, corners_right, ret_right
                )
            
            # Display
            display = np.hstack([display_left, display_right])
            cv2.putText(display, f"Captured: {count}/{num_images}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            if ret_left and ret_right:
                cv2.putText(display, "Press SPACE to capture", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(display, "Checkerboard not detected", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow('Calibration', display)
            
            key = cv2.waitKey(1) & 0xFF
            
            # Capture on SPACE
            if key == ord(' ') and ret_left and ret_right:
                # Save images
                cv2.imwrite(f'{output_dir}/left_{count:02d}.png', frame_left)
                cv2.imwrite(f'{output_dir}/right_{count:02d}.png', frame_right)
                count += 1
                print(f"Captured image pair {count}/{num_images}")
            
            # Exit on ESC
            elif key == 27:
                break
        
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()
        
        print(f"\nCaptured {count} image pairs")
        print(f"Images saved to {output_dir}/")
    
    def calibrate_from_images(self, image_dir='calib_images'):
        """
        Perform stereo calibration from saved images
        
        Args:
            image_dir: Directory containing calibration images
        """
        # Get image lists
        left_images = sorted(glob.glob(f'{image_dir}/left_*.png'))
        right_images = sorted(glob.glob(f'{image_dir}/right_*.png'))
        
        if len(left_images) == 0 or len(right_images) == 0:
            raise ValueError(f"No calibration images found in {image_dir}")
        
        if len(left_images) != len(right_images):
            raise ValueError("Number of left and right images don't match")
        
        print(f"\nProcessing {len(left_images)} image pairs...")
        
        for i, (left_path, right_path) in enumerate(zip(left_images, right_images)):
            # Read images
            img_left = cv2.imread(left_path)
            img_right = cv2.imread(right_path)
            
            if self.image_size is None:
                self.image_size = (img_left.shape[1], img_left.shape[0])
            
            # Convert to grayscale
            gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret_left, corners_left = cv2.findChessboardCorners(
                gray_left, self.checkerboard_size, None
            )
            ret_right, corners_right = cv2.findChessboardCorners(
                gray_right, self.checkerboard_size, None
            )
            
            if ret_left and ret_right:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                
                corners_left = cv2.cornerSubPix(
                    gray_left, corners_left, (11, 11), (-1, -1), criteria
                )
                corners_right = cv2.cornerSubPix(
                    gray_right, corners_right, (11, 11), (-1, -1), criteria
                )
                
                # Store points
                self.objpoints.append(self.objp)
                self.imgpoints_left.append(corners_left)
                self.imgpoints_right.append(corners_right)
                
                print(f"  ✓ Image pair {i+1}/{len(left_images)}")
            else:
                print(f"  ✗ Image pair {i+1}/{len(left_images)} - corners not found")
        
        if len(self.objpoints) < 10:
            raise ValueError(f"Not enough valid image pairs: {len(self.objpoints)}")
        
        print(f"\nCalibrating with {len(self.objpoints)} valid image pairs...")
        
        # Calibrate individual cameras
        print("  Calibrating left camera...")
        ret_left, K1, D1, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, self.image_size, None, None
        )
        
        print("  Calibrating right camera...")
        ret_right, K2, D2, rvecs_right, tvecs_right = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, self.image_size, None, None
        )
        
        # Stereo calibration
        print("  Performing stereo calibration...")
        flags = cv2.CALIB_FIX_INTRINSIC
        
        criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        
        ret_stereo, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints,
            self.imgpoints_left,
            self.imgpoints_right,
            K1, D1,
            K2, D2,
            self.image_size,
            criteria=criteria_stereo,
            flags=flags
        )
        
        print(f"\n✓ Calibration completed!")
        print(f"  Stereo reprojection error: {ret_stereo:.4f} pixels")
        print(f"  Baseline: {abs(T[0]):.4f} meters")
        
        return K1, D1, K2, D2, R, T
    
    def save_calibration(self, K1, D1, K2, D2, R, T, filename='calibration.npz'):
        """Save calibration to file"""
        np.savez(
            filename,
            K1=K1, D1=D1,
            K2=K2, D2=D2,
            R=R, T=T,
            image_size=self.image_size
        )
        print(f"\n✓ Calibration saved to {filename}")
    
    def test_rectification(self, calibration_file='calibration.npz', test_image_dir='calib_images'):
        """Test rectification with calibration"""
        # Load calibration
        calib = np.load(calibration_file)
        K1 = calib['K1']
        D1 = calib['D1']
        K2 = calib['K2']
        D2 = calib['D2']
        R = calib['R']
        T = calib['T']
        image_size = tuple(calib['image_size'])
        
        # Compute rectification
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, image_size, R, T, alpha=0
        )
        
        map1_left, map2_left = cv2.initUndistortRectifyMap(
            K1, D1, R1, P1, image_size, cv2.CV_16SC2
        )
        map1_right, map2_right = cv2.initUndistortRectifyMap(
            K2, D2, R2, P2, image_size, cv2.CV_16SC2
        )
        
        # Test on first image pair
        left_image = cv2.imread(f'{test_image_dir}/left_00.png')
        right_image = cv2.imread(f'{test_image_dir}/right_00.png')
        
        if left_image is None or right_image is None:
            print("Test images not found")
            return
        
        # Rectify
        rect_left = cv2.remap(left_image, map1_left, map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(right_image, map1_right, map2_right, cv2.INTER_LINEAR)
        
        # Draw horizontal lines
        for y in range(0, rect_left.shape[0], 50):
            cv2.line(rect_left, (0, y), (rect_left.shape[1], y), (0, 255, 0), 1)
            cv2.line(rect_right, (0, y), (rect_right.shape[1], y), (0, 255, 0), 1)
        
        # Display
        display = np.hstack([rect_left, rect_right])
        cv2.imwrite('rectification_test.jpg', display)
        
        print("\n✓ Rectification test saved to rectification_test.jpg")
        print("  Horizontal lines should align between left and right images")


def main():
    parser = argparse.ArgumentParser(description='Stereo Camera Calibration')
    parser.add_argument('--capture', action='store_true', 
                       help='Capture calibration images')
    parser.add_argument('--calibrate', action='store_true', 
                       help='Perform calibration from images')
    parser.add_argument('--test', action='store_true', 
                       help='Test rectification')
    parser.add_argument('--num-images', type=int, default=20,
                       help='Number of image pairs to capture (default: 20)')
    parser.add_argument('--left-camera', type=int, default=0,
                       help='Left camera device ID (default: 0)')
    parser.add_argument('--right-camera', type=int, default=1,
                       help='Right camera device ID (default: 1)')
    parser.add_argument('--checkerboard', type=str, default='9,6',
                       help='Checkerboard size as width,height (default: 9,6)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='Checkerboard square size in meters (default: 0.025)')
    parser.add_argument('--output', type=str, default='calibration.npz',
                       help='Output calibration file (default: calibration.npz)')
    
    args = parser.parse_args()
    
    # Parse checkerboard size
    checkerboard_size = tuple(map(int, args.checkerboard.split(',')))
    
    # Create calibration object
    calib = StereoCalibration(
        checkerboard_size=checkerboard_size,
        square_size=args.square_size,
        left_camera_id=args.left_camera,
        right_camera_id=args.right_camera
    )
    
    if args.capture:
        calib.capture_calibration_images(num_images=args.num_images)
    
    if args.calibrate:
        K1, D1, K2, D2, R, T = calib.calibrate_from_images()
        calib.save_calibration(K1, D1, K2, D2, R, T, filename=args.output)
    
    if args.test:
        calib.test_rectification(calibration_file=args.output)
    
    if not (args.capture or args.calibrate or args.test):
        print("Please specify --capture, --calibrate, or --test")
        print("For full calibration workflow:")
        print("  1. python3 calibrate_stereo.py --capture")
        print("  2. python3 calibrate_stereo.py --calibrate")
        print("  3. python3 calibrate_stereo.py --test")


if __name__ == '__main__':
    main()
