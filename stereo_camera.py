"""
Stereo Camera Module for Visual Odometry
Supports dual USB cameras with calibration data
"""
import cv2
import numpy as np
import logging
import time
from typing import Optional, Tuple
import threading

logger = logging.getLogger(__name__)


class StereoCamera:
    """
    Stereo camera system using two USB cameras
    """
    
    def __init__(self, 
                 left_cam_id: int = 0,
                 right_cam_id: int = 1,
                 calibration_file: str = "calibration.npz",
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30):
        
        self.left_cam_id = left_cam_id
        self.right_cam_id = right_cam_id
        self.width = width
        self.height = height
        self.fps = fps
        
        # Camera objects
        self.left_cam = None
        self.right_cam = None
        
        # Calibration parameters
        self.K_left = None
        self.D_left = None
        self.K_right = None
        self.D_right = None
        self.R = None
        self.T = None
        self.rectify_maps_left = None
        self.rectify_maps_right = None
        
        # Stereo matcher
        self.stereo_matcher = None
        
        # Threading
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.latest_left = None
        self.latest_right = None
        self.latest_disparity = None
        
        # Load calibration
        self.load_calibration(calibration_file)
        
    def load_calibration(self, filepath: str) -> bool:
        """Load camera calibration from .npz file"""
        try:
            if not os.path.exists(filepath):
                logger.warning(f"Calibration file not found: {filepath}, using defaults")
                return self._use_default_calibration()
            
            data = np.load(filepath)
            
            # Check if required keys exist
            required_keys = ['K_left', 'D_left', 'K_right', 'D_right', 'R', 'T']
            missing_keys = [key for key in required_keys if key not in data.files]
            
            if missing_keys:
                logger.warning(f"Calibration missing keys: {missing_keys}, using defaults")
                return self._use_default_calibration()
            
            self.K_left = data['K_left']
            self.D_left = data['D_left']
            self.K_right = data['K_right']
            self.D_right = data['D_right']
            self.R = data['R']
            self.T = data['T']
            
            # Compute rectification maps
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                self.K_left, self.D_left,
                self.K_right, self.D_right,
                (self.width, self.height),
                self.R, self.T,
                alpha=0
            )
            
            self.rectify_maps_left = cv2.initUndistortRectifyMap(
                self.K_left, self.D_left, R1, P1,
                (self.width, self.height), cv2.CV_16SC2
            )
            
            self.rectify_maps_right = cv2.initUndistortRectifyMap(
                self.K_right, self.D_right, R2, P2,
                (self.width, self.height), cv2.CV_16SC2
            )
            
            self.Q = Q  # Disparity-to-depth mapping matrix
            
            logger.info(f"Calibration loaded from {filepath}")
            logger.info(f"Baseline: {abs(self.T[0][0])*1000:.1f}mm")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            logger.warning("Using default calibration parameters")
            return self._use_default_calibration()
    
    def _use_default_calibration(self) -> bool:
        """Use default calibration parameters when calibration file is unavailable"""
        logger.info("Setting up default camera calibration")
        
        # Default camera matrix (typical webcam at 640x480)
        self.K_left = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        
        self.K_right = self.K_left.copy()
        
        # Zero distortion
        self.D_left = np.zeros(5, dtype=np.float64)
        self.D_right = np.zeros(5, dtype=np.float64)
        
        # Identity rotation (cameras are aligned)
        self.R = np.eye(3, dtype=np.float64)
        
        # 60mm baseline (typical stereo setup)
        self.T = np.array([[0.06], [0.0], [0.0]], dtype=np.float64)
        
        # Compute rectification with default parameters
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            self.K_left, self.D_left,
            self.K_right, self.D_right,
            (self.width, self.height),
            self.R, self.T,
            alpha=0
        )
        
        self.rectify_maps_left = cv2.initUndistortRectifyMap(
            self.K_left, self.D_left, R1, P1,
            (self.width, self.height), cv2.CV_16SC2
        )
        
        self.rectify_maps_right = cv2.initUndistortRectifyMap(
            self.K_right, self.D_right, R2, P2,
            (self.width, self.height), cv2.CV_16SC2
        )
        
        self.Q = Q
        
        logger.warning("Using UNCALIBRATED default parameters - depth/scale will be inaccurate!")
        logger.info("Please run calibration with calibrate_stereo.py for accurate measurements")
        
        return True
    
    def init_cameras(self) -> bool:
        """Initialize both cameras"""
        try:
            # Initialize left camera
            self.left_cam = cv2.VideoCapture(self.left_cam_id)
            self.left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.left_cam.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Initialize right camera
            self.right_cam = cv2.VideoCapture(self.right_cam_id)
            self.right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.right_cam.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Check if cameras opened
            if not self.left_cam.isOpened():
                logger.error(f"Failed to open left camera {self.left_cam_id}")
                return False
                
            if not self.right_cam.isOpened():
                logger.error(f"Failed to open right camera {self.right_cam_id}")
                return False
            
            # Initialize stereo matcher (StereoSGBM)
            self.stereo_matcher = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=128,  # Must be divisible by 16
                blockSize=5,
                P1=8 * 3 * 5**2,
                P2=32 * 3 * 5**2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
            
            logger.info("Stereo cameras initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Camera initialization error: {e}")
            return False
    
    def capture_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Capture synchronized frames from both cameras"""
        if not self.left_cam or not self.right_cam:
            return None, None
        
        ret_l, frame_l = self.left_cam.read()
        ret_r, frame_r = self.right_cam.read()
        
        if not ret_l or not ret_r:
            logger.warning("Failed to capture frames")
            return None, None
        
        return frame_l, frame_r
    
    def rectify_frames(self, left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Apply rectification to stereo pair"""
        if self.rectify_maps_left is None or self.rectify_maps_right is None:
            return left, right
        
        left_rect = cv2.remap(left, self.rectify_maps_left[0], 
                              self.rectify_maps_left[1], cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.rectify_maps_right[0], 
                               self.rectify_maps_right[1], cv2.INTER_LINEAR)
        
        return left_rect, right_rect
    
    def compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Compute disparity map from rectified stereo pair"""
        if self.stereo_matcher is None:
            return None
        
        # Convert to grayscale if needed
        if len(left.shape) == 3:
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left
            right_gray = right
        
        # Compute disparity
        disparity = self.stereo_matcher.compute(left_gray, right_gray)
        disparity = disparity.astype(np.float32) / 16.0  # Convert to float
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """Convert disparity map to depth map using Q matrix"""
        if self.Q is None:
            return None
        
        # Reproject to 3D
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        
        # Extract depth (Z coordinate)
        depth = points_3d[:, :, 2]
        
        return depth
    
    def _capture_loop(self):
        """Background thread for continuous frame capture"""
        while self.running:
            left, right = self.capture_frames()
            
            if left is not None and right is not None:
                # Rectify
                left_rect, right_rect = self.rectify_frames(left, right)
                
                # Compute disparity
                disparity = self.compute_disparity(left_rect, right_rect)
                
                with self.lock:
                    self.latest_left = left_rect
                    self.latest_right = right_rect
                    self.latest_disparity = disparity
            
            time.sleep(0.01)  # ~100 FPS max
    
    def start_capture(self):
        """Start background capture thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.thread.start()
            logger.info("Stereo capture thread started")
    
    def stop_capture(self):
        """Stop background capture thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        logger.info("Stereo capture thread stopped")
    
    def get_latest_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Get latest captured frames (thread-safe)"""
        with self.lock:
            return self.latest_left, self.latest_right, self.latest_disparity
    
    def release(self):
        """Release camera resources"""
        self.stop_capture()
        
        if self.left_cam:
            self.left_cam.release()
        if self.right_cam:
            self.right_cam.release()
        
        logger.info("Cameras released")


class VisualOdometry:
    """
    Visual Odometry using stereo camera
    Estimates camera motion from consecutive frames
    """
    
    def __init__(self, stereo_camera: StereoCamera):
        self.camera = stereo_camera
        
        # ORB feature detector
        self.detector = cv2.ORB_create(nfeatures=2000)
        
        # Feature matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # Current pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Statistics
        self.frame_count = 0
        self.total_distance = 0.0
        
        logger.info("Visual Odometry initialized")
    
    def detect_features(self, frame: np.ndarray) -> Tuple[list, np.ndarray]:
        """Detect ORB features in frame"""
        keypoints, descriptors = self.detector.detectAndCompute(frame, None)
        return keypoints, descriptors
    
    def match_features(self, desc1: np.ndarray, desc2: np.ndarray, ratio: float = 0.75):
        """Match features between two frames using Lowe's ratio test"""
        if desc1 is None or desc2 is None:
            return []
        
        # Use KNN matching
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        # Apply ratio test
        good_matches = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < ratio * n.distance:
                    good_matches.append(m)
        
        return good_matches
    
    def estimate_motion(self, kp1: list, kp2: list, matches: list, 
                       disparity: np.ndarray) -> Tuple[float, float, float]:
        """
        Estimate camera motion from matched features
        
        Returns:
            dx, dy, dtheta: Translation and rotation
        """
        if len(matches) < 8:
            return 0.0, 0.0, 0.0
        
        # Extract matched point coordinates
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
        
        # Estimate essential matrix (camera motion)
        E, mask = cv2.findEssentialMat(
            pts1, pts2, 
            self.camera.K_left, 
            method=cv2.RANSAC, 
            prob=0.999, 
            threshold=1.0
        )
        
        if E is None:
            return 0.0, 0.0, 0.0
        
        # Recover pose from essential matrix
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.camera.K_left)
        
        # Extract translation (scale is unknown without absolute depth)
        # Use disparity to estimate scale
        dx = t[0][0] * 0.1  # Scale factor (tune based on actual motion)
        dy = t[2][0] * 0.1  # Forward/backward
        
        # Extract rotation around Z-axis (yaw)
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        
        return dx, dy, dtheta
    
    def update(self, frame: np.ndarray, disparity: np.ndarray) -> Tuple[float, float, float]:
        """
        Process new frame and update pose estimate
        
        Returns:
            Current pose (x, y, theta)
        """
        # Detect features
        keypoints, descriptors = self.detect_features(frame)
        
        if self.prev_descriptors is not None:
            # Match with previous frame
            matches = self.match_features(self.prev_descriptors, descriptors)
            
            if len(matches) > 8:
                # Estimate motion
                dx, dy, dtheta = self.estimate_motion(
                    self.prev_keypoints, keypoints, matches, disparity
                )
                
                # Update pose
                self.x += dx * np.cos(self.theta) - dy * np.sin(self.theta)
                self.y += dx * np.sin(self.theta) + dy * np.cos(self.theta)
                self.theta += dtheta
                
                # Normalize angle
                self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
                
                # Update statistics
                distance = np.sqrt(dx**2 + dy**2)
                self.total_distance += distance
        
        # Store current frame data
        self.prev_frame = frame.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.frame_count += 1
        
        return self.x, self.y, self.theta
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose estimate"""
        return self.x, self.y, self.theta
    
    def reset(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.frame_count = 0
        self.total_distance = 0.0
        logger.info("Visual odometry reset")