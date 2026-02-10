"""
ORB-SLAM3 Integration Module
Wrapper for ORB-SLAM3 C++ executable with Python interface
"""
import subprocess
import numpy as np
import os
import threading
import time
import logging
from typing import Optional, Tuple
from queue import Queue

logger = logging.getLogger(__name__)


class ORBSLAM3:
    """
    Python wrapper for ORB-SLAM3 system
    Runs as subprocess and parses trajectory output
    """
    
    def __init__(self, 
                 orbslam_path: str = "/home/jetson/ORB_SLAM3",
                 vocabulary_path: str = None,
                 settings_path: str = None,
                 mode: str = "stereo"):
        
        self.orbslam_path = orbslam_path
        self.mode = mode
        
        # Default paths
        if vocabulary_path is None:
            vocabulary_path = os.path.join(orbslam_path, "Vocabulary/ORBvoc.txt")
        if settings_path is None:
            settings_path = "orbslam_stereo.yaml"
        
        self.vocabulary_path = vocabulary_path
        self.settings_path = settings_path
        
        # Process handle
        self.process = None
        self.running = False
        
        # State
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.tracking_state = "NOT_INITIALIZED"
        
        # Trajectory log
        self.trajectory = []
        self.trajectory_file = "trajectory.txt"
        
        # Input queues
        self.frame_queue = Queue(maxsize=5)
        self.process_thread = None
        
    def create_settings_file(self, camera_info: dict):
        """
        Create ORB-SLAM3 settings YAML file from camera calibration
        
        Args:
            camera_info: Dict with K1, D1, K2, D2, baseline, width, height, fps
        """
        yaml_content = f"""
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------
Camera.type: "PinHole"

# Left Camera calibration
Camera1.fx: {camera_info['K1'][0, 0]}
Camera1.fy: {camera_info['K1'][1, 1]}
Camera1.cx: {camera_info['K1'][0, 2]}
Camera1.cy: {camera_info['K1'][1, 2]}

Camera1.k1: {camera_info['D1'][0]}
Camera1.k2: {camera_info['D1'][1]}
Camera1.p1: {camera_info['D1'][2]}
Camera1.p2: {camera_info['D1'][3]}

# Right Camera calibration
Camera2.fx: {camera_info['K2'][0, 0]}
Camera2.fy: {camera_info['K2'][1, 1]}
Camera2.cx: {camera_info['K2'][0, 2]}
Camera2.cy: {camera_info['K2'][1, 2]}

Camera2.k1: {camera_info['D2'][0]}
Camera2.k2: {camera_info['D2'][1]}
Camera2.p1: {camera_info['D2'][2]}
Camera2.p2: {camera_info['D2'][3]}

# Stereo baseline * fx
Camera.bf: {abs(camera_info['baseline']) * camera_info['K1'][0, 0]}

# Image resolution
Camera.width: {camera_info['width']}
Camera.height: {camera_info['height']}

# Frame rate
Camera.fps: {camera_info['fps']}

# Color order (0: BGR, 1: RGB)
Camera.RGB: 0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1200

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold (lower = more features)
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

#--------------------------------------------------------------------------------------------
# Map Parameters
#--------------------------------------------------------------------------------------------
# Close/Far threshold for point culling
ThDepth: 40.0
"""
        
        with open(self.settings_path, 'w') as f:
            f.write(yaml_content)
        
        logger.info(f"Created ORB-SLAM3 settings file: {self.settings_path}")
    
    def start(self, use_viewer: bool = False):
        """
        Start ORB-SLAM3 process
        
        Args:
            use_viewer: Enable GUI viewer (requires X display)
        """
        if self.running:
            logger.warning("ORB-SLAM3 already running")
            return
        
        # Build command
        executable = os.path.join(self.orbslam_path, "Examples/Stereo/stereo_euroc")
        
        if not os.path.exists(executable):
            logger.error(f"ORB-SLAM3 executable not found: {executable}")
            return
        
        cmd = [
            executable,
            self.vocabulary_path,
            self.settings_path
        ]
        
        if not use_viewer:
            cmd.append("--no-viewer")
        
        # Start process
        try:
            self.process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            
            self.running = True
            
            # Start processing thread
            self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
            self.process_thread.start()
            
            logger.info("ORB-SLAM3 started")
            
        except Exception as e:
            logger.error(f"Failed to start ORB-SLAM3: {e}")
    
    def stop(self):
        """Stop ORB-SLAM3 process"""
        self.running = False
        
        if self.process:
            self.process.terminate()
            self.process.wait(timeout=5)
            logger.info("ORB-SLAM3 stopped")
    
    def _process_loop(self):
        """Process stdout/stderr from ORB-SLAM3"""
        while self.running and self.process:
            # Read output (non-blocking would be better)
            try:
                output = self.process.stdout.readline().decode('utf-8').strip()
                if output:
                    self._parse_output(output)
            except:
                pass
            
            time.sleep(0.001)
    
    def _parse_output(self, line: str):
        """Parse ORB-SLAM3 output for pose/state"""
        # Example parsing - adjust based on actual ORB-SLAM3 output format
        if "Tracking" in line:
            if "LOST" in line:
                self.tracking_state = "LOST"
            elif "OK" in line:
                self.tracking_state = "OK"
        
        # Parse pose if available (format depends on ORB-SLAM3 version)
        # This is a placeholder - actual parsing needed
        pass
    
    def track_stereo(self, img_left: np.ndarray, img_right: np.ndarray, timestamp: float) -> Optional[np.ndarray]:
        """
        Track stereo frame pair
        
        Returns:
            4x4 camera pose matrix (if tracking successful), None otherwise
        """
        if not self.running:
            return None
        
        # For subprocess-based ORB-SLAM3, we need to use shared memory or file I/O
        # Alternative: Use ORB-SLAM3 Python bindings if available
        
        # Placeholder: In practice, you'd either:
        # 1. Use ORB-SLAM3 Python bindings (recommended)
        # 2. Use shared memory (complex)
        # 3. Save images to disk and use file-based input (slow)
        
        # For now, return identity (no actual SLAM)
        logger.debug("SLAM tracking called (placeholder)")
        return self.current_pose
    
    def get_pose(self) -> Tuple[np.ndarray, str]:
        """
        Get current camera pose
        
        Returns:
            (pose_matrix, tracking_state)
        """
        return self.current_pose.copy(), self.tracking_state
    
    def save_trajectory(self, filename: str = None):
        """Save trajectory to file"""
        if filename is None:
            filename = self.trajectory_file
        
        with open(filename, 'w') as f:
            for timestamp, pose in self.trajectory:
                # Save in TUM format: timestamp tx ty tz qx qy qz qw
                f.write(f"{timestamp} {pose[0,3]} {pose[1,3]} {pose[2,3]} 0 0 0 1\n")
        
        logger.info(f"Trajectory saved to {filename}")


class SimpleVisualOdometry:
    """
    Simple visual odometry fallback (when ORB-SLAM3 is not available)
    Uses feature tracking for relative motion estimation
    """
    
    def __init__(self):
        self.prev_left = None
        self.prev_right = None
        
        # Feature detector
        self.detector = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Current pose
        self.pose = np.eye(4)
        
    def track(self, img_left: np.ndarray, img_right: np.ndarray) -> np.ndarray:
        """
        Track motion between frames
        
        Returns:
            Current pose estimate
        """
        if self.prev_left is None:
            self.prev_left = img_left
            self.prev_right = img_right
            return self.pose
        
        # Detect and match features
        kp1, des1 = self.detector.detectAndCompute(self.prev_left, None)
        kp2, des2 = self.detector.detectAndCompute(img_left, None)
        
        if des1 is not None and des2 is not None:
            matches = self.matcher.match(des1, des2)
            
            # Estimate motion (simplified)
            if len(matches) > 10:
                # Extract matched points
                pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
                
                # Estimate essential matrix (would need camera matrix)
                # For now, just estimate 2D motion
                dx = np.median(pts2[:, 0] - pts1[:, 0])
                dy = np.median(pts2[:, 1] - pts1[:, 1])
                
                # Update pose (very rough estimate)
                self.pose[0, 3] += dx * 0.001
                self.pose[1, 3] += dy * 0.001
        
        self.prev_left = img_left
        self.prev_right = img_right
        
        return self.pose
