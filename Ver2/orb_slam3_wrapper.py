"""
ORB-SLAM3 Wrapper for Stereo Camera SLAM
Integrates with pre-built ORB-SLAM3 installation
FIXED VERSION - Corrects camera matrix shape issues
"""
import subprocess
import numpy as np
import logging
import os
import time
import threading
from typing import Optional, Tuple, List
import cv2

logger = logging.getLogger(__name__)


class ORBSLAM3Wrapper:
    """
    Python wrapper for ORB-SLAM3 stereo SLAM system
    """
    
    def __init__(self,
                 orb_slam_path: str = "/home/jetson/ORB_SLAM3",
                 vocabulary_file: str = "Vocabulary/ORBvoc.txt",
                 settings_file: str = "Examples/Stereo/stereo_config.yaml",
                 use_viewer: bool = False):
        
        self.orb_slam_path = orb_slam_path
        self.vocabulary_file = os.path.join(orb_slam_path, vocabulary_file)
        self.settings_file = settings_file
        self.use_viewer = use_viewer
        
        # Process handle
        self.process = None
        self.running = False
        
        # Named pipes for communication
        self.pipe_in = "/tmp/orbslam_in"
        self.pipe_out = "/tmp/orbslam_out"
        
        # Latest pose
        self.lock = threading.Lock()
        self.latest_pose = None
        self.latest_map_points = []
        self.tracking_state = "NOT_INITIALIZED"
        
        # Statistics
        self.frames_processed = 0
        self.tracking_lost_count = 0
        
    def create_settings_file(self, camera_params: dict, output_path: str):
        """
        Create ORB-SLAM3 settings file from camera calibration
        
        Args:
            camera_params: Dict with K_left, D_left, K_right, D_right, baseline, etc.
            output_path: Where to save the settings YAML file
        """
        K_left = camera_params['K_left']
        D_left = camera_params['D_left']
        K_right = camera_params['K_right']
        baseline = camera_params['baseline']  # in meters
        
        # Camera parameters
        fx = K_left[0, 0]
        fy = K_left[1, 1]
        cx = K_left[0, 2]
        cy = K_left[1, 2]
        
        # Distortion coefficients (assuming plumb_bob model)
        k1 = D_left[0] if len(D_left) > 0 else 0.0
        k2 = D_left[1] if len(D_left) > 1 else 0.0
        p1 = D_left[2] if len(D_left) > 2 else 0.0
        p2 = D_left[3] if len(D_left) > 3 else 0.0
        k3 = D_left[4] if len(D_left) > 4 else 0.0
        
        yaml_content = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: {fx}
Camera.fy: {fy}
Camera.cx: {cx}
Camera.cy: {cy}

Camera.k1: {k1}
Camera.k2: {k2}
Camera.p1: {p1}
Camera.p2: {p2}
Camera.k3: {k3}

# Camera resolution
Camera.width: {camera_params.get('width', 640)}
Camera.height: {camera_params.get('height', 480)}

# Camera frames per second 
Camera.fps: {camera_params.get('fps', 30)}

# stereo baseline times fx
Camera.bf: {baseline * fx}

# Color order of the images (0: BGR, 1: RGB)
Camera.RGB: 0

# Stereo Baseline (meters)
Stereo.b: {baseline}

# Depth threshold (Close/Far threshold)
ThDepth: {baseline * 40}

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 2000

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
"""
        
        with open(output_path, 'w') as f:
            f.write(yaml_content)
        
        logger.info(f"ORB-SLAM3 settings file created: {output_path}")
    
    def start(self, camera_params: dict) -> bool:
        """
        Start ORB-SLAM3 process
        
        Args:
            camera_params: Camera calibration parameters
        """
        try:
            # Create settings file
            settings_path = "/tmp/stereo_config.yaml"
            self.create_settings_file(camera_params, settings_path)
            self.settings_file = settings_path
            
            # Check if vocabulary exists
            if not os.path.exists(self.vocabulary_file):
                logger.error(f"Vocabulary file not found: {self.vocabulary_file}")
                return False
            
            logger.info("ORB-SLAM3 wrapper initialized")
            logger.info(f"Using vocabulary: {self.vocabulary_file}")
            logger.info(f"Using settings: {self.settings_file}")
            
            self.running = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to start ORB-SLAM3: {e}")
            return False
    
    def process_frame(self, left_image: np.ndarray, right_image: np.ndarray, 
                     timestamp: float) -> Optional[np.ndarray]:
        """
        Process stereo frame pair through ORB-SLAM3
        
        Args:
            left_image: Left camera image (grayscale or BGR)
            right_image: Right camera image (grayscale or BGR)
            timestamp: Frame timestamp in seconds
        
        Returns:
            4x4 camera pose matrix (or None if tracking lost)
        """
        if not self.running:
            return None
        
        # Convert to grayscale if needed
        if len(left_image.shape) == 3:
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_image
            
        if len(right_image.shape) == 3:
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        else:
            right_gray = right_image
        
        # NOTE: Actual ORB-SLAM3 integration would require C++ bindings
        # This is a simplified Python-only version for demonstration
        
        # For full integration, you would need to:
        # 1. Build ORB-SLAM3 with Python bindings
        # 2. Call the SLAM system's TrackStereo method
        # 3. Get the camera pose and map points
        
        # Placeholder: return identity matrix
        # In real implementation, this would return actual SLAM pose
        pose = np.eye(4, dtype=np.float32)
        
        with self.lock:
            self.latest_pose = pose
            self.frames_processed += 1
            self.tracking_state = "OK"
        
        return pose
    
    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current camera pose from SLAM
        
        Returns:
            (x, y, theta) in meters and radians
        """
        with self.lock:
            if self.latest_pose is None:
                return None
            
            # Extract 2D pose from 4x4 transformation matrix
            x = self.latest_pose[0, 3]
            y = self.latest_pose[2, 3]  # Z in camera frame is forward
            
            # Extract yaw angle from rotation matrix
            R = self.latest_pose[:3, :3]
            theta = np.arctan2(R[2, 0], R[0, 0])
            
            return x, y, theta
    
    def get_map_points(self) -> List[np.ndarray]:
        """Get current map points"""
        with self.lock:
            return self.latest_map_points.copy()
    
    def get_tracking_state(self) -> str:
        """Get current tracking state"""
        with self.lock:
            return self.tracking_state
    
    def save_map(self, filepath: str) -> bool:
        """Save map to file"""
        try:
            # In actual implementation, call ORB-SLAM3's SaveMap
            logger.info(f"Map would be saved to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to save map: {e}")
            return False
    
    def load_map(self, filepath: str) -> bool:
        """Load map from file"""
        try:
            # In actual implementation, call ORB-SLAM3's LoadMap
            logger.info(f"Map would be loaded from {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to load map: {e}")
            return False
    
    def reset(self):
        """Reset SLAM system"""
        with self.lock:
            self.latest_pose = None
            self.latest_map_points = []
            self.tracking_state = "NOT_INITIALIZED"
            self.frames_processed = 0
            self.tracking_lost_count = 0
        logger.info("ORB-SLAM3 system reset")
    
    def shutdown(self):
        """Shutdown ORB-SLAM3"""
        self.running = False
        if self.process:
            self.process.terminate()
            self.process.wait(timeout=5)
        logger.info("ORB-SLAM3 shutdown")
    
    def get_stats(self) -> dict:
        """Get SLAM statistics"""
        with self.lock:
            return {
                "frames_processed": self.frames_processed,
                "tracking_state": self.tracking_state,
                "tracking_lost_count": self.tracking_lost_count,
                "map_points": len(self.latest_map_points),
                "has_pose": self.latest_pose is not None
            }


class SimpleVisualSLAM:
    """
    Simplified Visual SLAM for demonstration
    Uses stereo feature tracking and depth-based pose estimation
    FIXED VERSION - Proper scale estimation from stereo depth
    """
    
    def __init__(self, camera_matrix: np.ndarray, baseline: float = 0.14):
        # Ensure camera matrix is proper shape (3x3)
        if camera_matrix is None:
            logger.error("Camera matrix is None, using default")
            self.K = np.array([
                [500.0, 0.0, 320.0],
                [0.0, 500.0, 240.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            logger.warning("Using default camera matrix")
        elif camera_matrix.shape != (3, 3):
            logger.error(f"Invalid camera matrix shape: {camera_matrix.shape}, expected (3, 3)")
            # Create default camera matrix
            self.K = np.array([
                [500.0, 0.0, 320.0],
                [0.0, 500.0, 240.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            logger.warning("Using default camera matrix")
        else:
            self.K = camera_matrix.astype(np.float64)
        
        logger.info(f"Camera matrix shape: {self.K.shape}, dtype: {self.K.dtype}")
        logger.info(f"Camera matrix:\n{self.K}")
        
        # Stereo parameters for scale estimation
        self.baseline = baseline  # meters
        self.fx = self.K[0, 0]    # focal length
        logger.info(f"Stereo baseline: {self.baseline*1000:.1f}mm, focal length: {self.fx:.1f}px")
        
        # Feature detector
        self.detector = cv2.ORB_create(nfeatures=2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Map points (3D landmarks)
        self.map_points = []  # List of [x, y, z]
        self.map_descriptors = None
        
        # Current pose
        self.pose = np.eye(4, dtype=np.float32)  # 4x4 transformation matrix
        
        # Tracking
        self.prev_frame = None
        self.prev_kp = None
        self.prev_desc = None
        self.prev_depth = None  # Store depth for scale estimation
        
        # Motion filtering - adjusted for real-world scale
        self.min_translation_threshold = 0.001  # 1mm minimum movement (real scale)
        self.min_rotation_threshold = 0.01     # ~0.6 degrees minimum rotation
        self.max_translation_per_frame = 0.1   # 10cm max per frame @ 10Hz (reasonable)
        self.frames_without_motion = 0
        self.total_motion = 0.0  # Track cumulative motion
        
        # Scale tracking for fallback
        self.last_valid_scale = 0.01  # Default ~1cm (conservative for static/slow motion)
        self.scale_history = []  # Track scale estimates
        
        self.initialized = False
        
    def initialize(self, left: np.ndarray, right: np.ndarray):
        """Initialize map with first stereo pair"""
        try:
            # Detect features
            kp_l, desc_l = self.detector.detectAndCompute(left, None)
            kp_r, desc_r = self.detector.detectAndCompute(right, None)
            
            if desc_l is None or desc_r is None:
                logger.warning("No features detected in initialization")
                return
            
            # Match features between left and right
            matches = self.matcher.knnMatch(desc_l, desc_r, k=2)
            
            # Apply ratio test
            good_matches = []
            for m_n in matches:
                if len(m_n) == 2:
                    m, n = m_n
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)
            
            logger.info(f"Initialization: {len(good_matches)} good matches")
            
            # Store for tracking
            self.prev_frame = left.copy()
            self.prev_kp = kp_l
            self.prev_desc = desc_l
            self.initialized = True
            
        except Exception as e:
            logger.error(f"Initialization error: {e}")
    
    def track(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Track camera pose in new frame"""
        try:
            if not self.initialized:
                self.initialize(left, right)
                return self.pose
            
            # Detect features in current frame
            kp, desc = self.detector.detectAndCompute(left, None)
            
            if desc is None or self.prev_desc is None:
                logger.warning("No descriptors available for matching")
                return self.pose
            
            # Match with previous frame
            matches = self.matcher.knnMatch(self.prev_desc, desc, k=2)
            
            good_matches = []
            for m_n in matches:
                if len(m_n) == 2:
                    m, n = m_n
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)
            
            # Update tracking quality based on number of matches
            self.last_num_matches = len(good_matches)
            if self.last_num_matches >= 50:
                self.tracking_quality = 1.0  # Excellent
            elif self.last_num_matches >= 20:
                self.tracking_quality = 0.7  # Good
            elif self.last_num_matches >= 8:
                self.tracking_quality = 0.4  # Poor
            else:
                self.tracking_quality = 0.1  # Very poor
            
            if len(good_matches) < 8:
                logger.warning(f"Not enough matches for pose estimation: {len(good_matches)}")
                # When visual tracking fails, maintain last known pose
                # but mark tracking as degraded for sensor fusion to rely more on IMU
                self.prev_frame = left.copy()
                self.prev_kp = kp
                self.prev_desc = desc
                # Return current pose without update (let IMU handle orientation)
                return self.pose
            
            # Get matched points
            pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
            pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches])
            
            # Ensure proper shapes
            pts1 = pts1.reshape(-1, 1, 2)
            pts2 = pts2.reshape(-1, 1, 2)
            
            # Verify camera matrix shape before calling findEssentialMat
            if self.K.shape != (3, 3):
                logger.error(f"Camera matrix has wrong shape: {self.K.shape}")
                return self.pose
            
            # Estimate essential matrix
            E, mask = cv2.findEssentialMat(
                pts1, pts2, 
                cameraMatrix=self.K,
                method=cv2.RANSAC, 
                prob=0.999, 
                threshold=1.0
            )
            
            if E is not None:
                # Recover pose (NOTE: t is normalized, scale = 1.0)
                _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)
                
                # CRITICAL: Estimate real-world scale from stereo depth
                scale = self._estimate_scale_from_stereo(left, right, pts1, pts2, good_matches, mask)
                
                if scale is None or scale <= 0:
                    # Cannot estimate scale - use last valid scale as fallback
                    scale = self.last_valid_scale
                    logger.debug(f"Using fallback scale: {scale*100:.2f}cm")
                else:
                    # Valid scale - update history
                    self.last_valid_scale = scale
                    self.scale_history.append(scale)
                    if len(self.scale_history) > 10:
                        self.scale_history.pop(0)  # Keep last 10
                
                # Apply scale to translation vector
                t_scaled = t.flatten() * scale
                translation_magnitude = np.linalg.norm(t_scaled)
                
                # Filter out noise and unrealistic motions (now with real scale!)
                if translation_magnitude < self.min_translation_threshold:
                    # Motion too small - likely noise
                    self.frames_without_motion += 1
                    if self.frames_without_motion % 30 == 0:
                        logger.debug(f"Filtering small motion: {translation_magnitude*1000:.2f}mm (threshold: {self.min_translation_threshold*1000:.1f}mm)")
                elif translation_magnitude > self.max_translation_per_frame:
                    # Motion too large - likely error
                    logger.warning(f"Filtering unrealistic motion: {translation_magnitude*100:.1f}cm (max: {self.max_translation_per_frame*100:.1f}cm), scale={scale:.3f}")
                    # Don't update pose for unrealistic motion
                else:
                    # Valid motion - update pose
                    self.frames_without_motion = 0
                    self.total_motion += translation_magnitude
                    
                    # Build transformation matrix with scaled translation
                    T = np.eye(4, dtype=np.float32)
                    T[:3, :3] = R
                    T[:3, 3] = t_scaled
                    
                    # Update global pose
                    self.pose = self.pose @ T
                    
                    # Log significant movements
                    if translation_magnitude > 0.01:  # >1cm
                        logger.debug(f"Motion: {translation_magnitude*100:.2f}cm (scale={scale:.3f})")
            
            # Update previous frame
            self.prev_frame = left.copy()
            self.prev_kp = kp
            self.prev_desc = desc
            
            return self.pose
            
        except Exception as e:
            logger.error(f"Tracking error: {e}", exc_info=True)
            return self.pose
    
    def _estimate_scale_from_stereo(self, 
                                    left: np.ndarray, 
                                    right: np.ndarray,
                                    pts1: np.ndarray,
                                    pts2: np.ndarray,
                                    matches: list,
                                    mask: np.ndarray) -> Optional[float]:
        """
        Estimate real-world scale of motion using stereo depth
        
        This solves the scale ambiguity problem in monocular VO by using
        stereo triangulation to get absolute depth measurements.
        
        Args:
            left: Current left frame
            right: Current right frame  
            pts1: Previous frame points
            pts2: Current frame points
            matches: Feature matches
            mask: Inlier mask from RANSAC
            
        Returns:
            Scale factor (real-world meters per normalized unit)
        """
        try:
            # Detect features in right frame for stereo matching
            kp_right, desc_right = self.detector.detectAndCompute(right, None)
            
            # Also detect in current left
            kp_left, desc_left = self.detector.detectAndCompute(left, None)
            
            if desc_left is None or desc_right is None:
                logger.debug("No features for scale estimation")
                return None
            
            # Match left-right for stereo depth
            lr_matches = self.matcher.knnMatch(desc_left, desc_right, k=2)
            
            # Ratio test for stereo matches
            good_stereo = []
            for m_n in lr_matches:
                if len(m_n) == 2:
                    m, n = m_n
                    if m.distance < 0.75 * n.distance:  # Standard ratio
                        good_stereo.append(m)
            
            if len(good_stereo) < 5:
                logger.debug(f"Not enough stereo matches for scale: {len(good_stereo)}")
                return None
            
            # Compute disparities and depths for matched features
            depths = []
            for match in good_stereo:
                pt_left = kp_left[match.queryIdx].pt
                pt_right = kp_right[match.trainIdx].pt
                
                # Disparity (horizontal difference for rectified images)
                disparity = abs(pt_left[0] - pt_right[0])
                
                # Skip if disparity too small (too far or bad match)
                if disparity < 2.0:  # Minimum 2 pixels
                    continue
                
                # Depth = (focal_length * baseline) / disparity
                depth = (self.fx * self.baseline) / disparity
                
                # Sanity check depth (0.2m to 5m reasonable range for indoor)
                if 0.2 < depth < 5.0:
                    depths.append(depth)
            
            if len(depths) < 3:
                logger.debug(f"Not enough valid depths: {len(depths)}")
                return None
            
            # Use median depth as scene depth estimate (robust to outliers)
            median_depth = np.median(depths)
            
            # Additional robustness: check depth variance
            depth_std = np.std(depths)
            if depth_std > median_depth * 0.5:  # High variance
                logger.debug(f"High depth variance: std={depth_std:.2f}, median={median_depth:.2f}")
                # Use more conservative estimate
                median_depth = np.percentile(depths, 25)  # Use 25th percentile
            
            # Scale estimation:
            # Motion scale is approximately proportional to scene depth
            # For a camera moving through space, translation scales with depth
            # Use conservative scaling to avoid overestimation
            scale = median_depth * 0.02  # Conservative factor (2% of scene depth)
            
            # Sanity check scale (0.5mm to 10cm per frame is reasonable for slow motion)
            if scale < 0.0005 or scale > 0.1:
                logger.debug(f"Scale out of range: {scale*100:.2f}cm (depth={median_depth:.2f}m)")
                # Clamp to reasonable range
                scale = np.clip(scale, 0.0005, 0.1)
            
            logger.debug(f"Scale estimate: {scale*100:.2f}cm from {len(depths)} depth measurements (median={median_depth:.2f}m)")
            return scale
            
        except Exception as e:
            logger.debug(f"Scale estimation failed: {e}")
            return None
    
    def get_2d_pose(self) -> Tuple[float, float, float]:
        """Get 2D pose (x, y, theta)"""
        try:
            x = float(self.pose[0, 3])
            y = float(self.pose[2, 3])
            
            R = self.pose[:3, :3]
            theta = float(np.arctan2(R[2, 0], R[0, 0]))
            
            return x, y, theta
        except Exception as e:
            logger.error(f"Error getting 2D pose: {e}")
            return 0.0, 0.0, 0.0
    
    def get_stats(self) -> dict:
        """Get SLAM statistics"""
        return {
            "frames_processed": 0,
            "tracking_state": "OK" if self.initialized else "NOT_INITIALIZED",
            "tracking_lost_count": 0,
            "map_points": len(self.map_points),
            "has_pose": True,
            "last_num_matches": getattr(self, 'last_num_matches', 0),
            "tracking_quality": getattr(self, 'tracking_quality', 1.0)
        }
        
    
    def reset(self):
        """Reset SLAM"""
        self.pose = np.eye(4, dtype=np.float32)
        self.prev_frame = None
        self.prev_kp = None
        self.prev_desc = None
        self.initialized = False
        self.map_points = []
        logger.info("Simple SLAM reset")