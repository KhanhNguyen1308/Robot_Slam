"""
Stereo Camera Module for OV9732 cameras
Handles capture, rectification, and feature extraction
"""
import cv2
import numpy as np
import threading
import time
import logging
from typing import Tuple, Optional
from queue import Queue

logger = logging.getLogger(__name__)


class StereoCamera:
    """
    Stereo camera system with calibration support
    Optimized for OV9732 cameras
    """
    
    def __init__(self, 
                 left_id: int = 0, 
                 right_id: int = 1,
                 calibration_file: str = "calibration.npz",
                 width: int = 1280,
                 height: int = 720,
                 fps: int = 30):
        
        self.left_id = left_id
        self.right_id = right_id
        self.width = width
        self.height = height
        self.fps = fps
        
        # Cameras
        self.cap_left = None
        self.cap_right = None
        
        # Calibration parameters
        self.calibration_loaded = False
        self.K1 = None  # Left camera matrix
        self.D1 = None  # Left distortion
        self.K2 = None  # Right camera matrix
        self.D2 = None  # Right distortion
        self.R = None   # Rotation matrix
        self.T = None   # Translation vector
        self.R1 = None  # Rectification rotation left
        self.R2 = None  # Rectification rotation right
        self.P1 = None  # Projection matrix left
        self.P2 = None  # Projection matrix right
        self.Q = None   # Disparity-to-depth mapping
        
        # Rectification maps
        self.map1_left = None
        self.map2_left = None
        self.map1_right = None
        self.map2_right = None
        
        # Frame buffers
        self.frame_queue = Queue(maxsize=2)
        self.running = False
        self.capture_thread = None
        
        # Load calibration
        self.load_calibration(calibration_file)
        
    def load_calibration(self, calibration_file: str) -> bool:
        """Load stereo calibration from .npz file"""
        try:
            calib = np.load(calibration_file)
            
            self.K1 = calib['K1']
            self.D1 = calib['D1']
            self.K2 = calib['K2']
            self.D2 = calib['D2']
            self.R = calib['R']
            self.T = calib['T']
            
            # Compute rectification transforms
            self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
                self.K1, self.D1,
                self.K2, self.D2,
                (self.width, self.height),
                self.R, self.T,
                alpha=0,  # 0 = crop to valid pixels
                newImageSize=(self.width, self.height)
            )
            
            # Precompute rectification maps
            self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
                self.K1, self.D1, self.R1, self.P1,
                (self.width, self.height), cv2.CV_16SC2
            )
            
            self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
                self.K2, self.D2, self.R2, self.P2,
                (self.width, self.height), cv2.CV_16SC2
            )
            
            self.calibration_loaded = True
            logger.info(f"Calibration loaded from {calibration_file}")
            logger.info(f"Baseline: {abs(self.T[0]):.4f}m")
            return True
            
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            self.calibration_loaded = False
            return False
    
    def open(self) -> bool:
        """Open camera devices"""
        try:
            import os
            
            # Open left camera
            self.cap_left = cv2.VideoCapture(self.left_id)
            self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap_left.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap_left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            # Fix power line frequency for left camera (50Hz for Vietnam)
            os.system(f"v4l2-ctl -d /dev/video{self.left_id} --set-ctrl=power_line_frequency=1")
            
            # Open right camera
            self.cap_right = cv2.VideoCapture(self.right_id)
            self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap_right.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap_right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            # Fix power line frequency for right camera (50Hz for Vietnam)
            os.system(f"v4l2-ctl -d /dev/video{self.right_id} --set-ctrl=power_line_frequency=1")
            
            if not self.cap_left.isOpened() or not self.cap_right.isOpened():
                raise RuntimeError("Failed to open cameras")
            
            logger.info(f"Cameras opened: {self.width}x{self.height}@{self.fps}fps")
            logger.info("Power line frequency set to 50Hz (Vietnam)")
            return True
            
        except Exception as e:
            logger.error(f"Camera open error: {e}")
            return False
    
    def close(self):
        """Close camera devices"""
        self.stop_capture()
        
        if self.cap_left:
            self.cap_left.release()
        if self.cap_right:
            self.cap_right.release()
        
        logger.info("Cameras closed")
    
    def start_capture(self):
        """Start background capture thread"""
        if self.running:
            return
        
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        logger.info("Capture thread started")
    
    def stop_capture(self):
        """Stop background capture thread"""
        self.running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        logger.info("Capture thread stopped")
    
    def _capture_loop(self):
        """Background thread for frame capture"""
        while self.running:
            ret_left, frame_left = self.cap_left.read()
            ret_right, frame_right = self.cap_right.read()
            
            if not ret_left or not ret_right:
                logger.warning("Frame capture failed")
                time.sleep(0.01)
                continue
            
            # Add timestamp
            timestamp = time.time()
            
            # Put in queue (drop old frames)
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass
            
            self.frame_queue.put((frame_left, frame_right, timestamp))
    
    def get_frames(self, timeout: float = 1.0) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Get latest stereo frame pair"""
        try:
            return self.frame_queue.get(timeout=timeout)
        except:
            return None
    
    def rectify_frames(self, frame_left: np.ndarray, frame_right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Rectify stereo pair using calibration"""
        if not self.calibration_loaded:
            return frame_left, frame_right
        
        rect_left = cv2.remap(frame_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        return rect_left, rect_right
    
    def compute_disparity(self, rect_left: np.ndarray, rect_right: np.ndarray) -> np.ndarray:
        """Compute disparity map using StereoBM"""
        # Convert to grayscale
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        
        # StereoBM parameters
        stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
        # Compute disparity
        disparity = stereo.compute(gray_left, gray_right)
        
        # Normalize for visualization
        disparity_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        return disparity_norm
    
    def get_camera_info(self) -> dict:
        """Get camera and calibration info"""
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "calibration_loaded": self.calibration_loaded,
            "baseline": abs(self.T[0]) if self.calibration_loaded else None,
            "left_camera": self.left_id,
            "right_camera": self.right_id
        }