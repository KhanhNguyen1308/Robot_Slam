"""
Main Robot Controller
Integrates all subsystems: Camera, SLAM, Motor Control, Web Server
"""
import numpy as np
import cv2
import time
import logging
import threading
import signal
import sys
from typing import Optional

from serial_controller import RP2040Controller, SafetyController
from stereo_camera import StereoCamera
from orbslam_interface import ORBSLAM3, SimpleVisualOdometry
from web_server import RobotWebServer

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RobotSystem:
    """
    Main robot system integrating all components
    """
    
    def __init__(self, config: dict):
        self.config = config
        
        # Components
        self.motor_controller = None
        self.safety_controller = None
        self.camera = None
        self.slam = None
        self.web_server = None
        
        # State
        self.running = False
        self.slam_pose = np.eye(4)
        self.slam_tracking_state = "NOT_INITIALIZED"
        
        # Current frames
        self.current_frame_left = None
        self.current_frame_right = None
        self.current_timestamp = 0
        
        # Trajectory
        self.trajectory = []
        
        # Performance metrics
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Main processing thread
        self.main_thread = None
        
    def initialize(self) -> bool:
        """Initialize all subsystems"""
        logger.info("Initializing robot system...")
        
        try:
            # 1. Initialize motor controller
            logger.info("Connecting to RP2040...")
            self.motor_controller = RP2040Controller(
                port=self.config.get('serial_port', '/dev/ttyACM0'),
                baudrate=self.config.get('serial_baudrate', 115200)
            )
            
            if not self.motor_controller.connect():
                logger.error("Failed to connect to RP2040")
                return False
            
            # Wrap with safety controller
            self.safety_controller = SafetyController(
                self.motor_controller,
                max_linear=self.config.get('max_linear_speed', 0.5),
                max_angular=self.config.get('max_angular_speed', 2.0)
            )
            
            # 2. Initialize stereo camera
            logger.info("Initializing cameras...")
            self.camera = StereoCamera(
                left_id=self.config.get('camera_left_id', 0),
                right_id=self.config.get('camera_right_id', 1),
                calibration_file=self.config.get('calibration_file', 'calibration.npz'),
                width=self.config.get('camera_width', 1280),
                height=self.config.get('camera_height', 720),
                fps=self.config.get('camera_fps', 30)
            )
            
            if not self.camera.open():
                logger.error("Failed to open cameras")
                return False
            
            self.camera.start_capture()
            
            # 3. Initialize ORB-SLAM3 (optional)
            use_orbslam = self.config.get('use_orbslam', True)
            
            if use_orbslam:
                logger.info("Initializing ORB-SLAM3...")
                
                self.slam = ORBSLAM3(
                    orbslam_path=self.config.get('orbslam_path', '/home/jetson/ORB_SLAM3'),
                    vocabulary_path=self.config.get('orbslam_vocabulary'),
                    settings_path=self.config.get('orbslam_settings', 'orbslam_stereo.yaml')
                )
                
                # Create settings file from camera calibration
                camera_info = {
                    'K1': self.camera.K1,
                    'D1': self.camera.D1,
                    'K2': self.camera.K2,
                    'D2': self.camera.D2,
                    'baseline': abs(self.camera.T[0]),
                    'width': self.camera.width,
                    'height': self.camera.height,
                    'fps': self.camera.fps
                }
                self.slam.create_settings_file(camera_info)
                
                # Start SLAM
                self.slam.start(use_viewer=self.config.get('orbslam_viewer', False))
            else:
                logger.info("Using simple visual odometry")
                self.slam = SimpleVisualOdometry()
            
            # 4. Initialize web server
            logger.info("Starting web server...")
            self.web_server = RobotWebServer(
                robot_controller=self,
                stereo_camera=self.camera,
                host=self.config.get('webserver_host', '0.0.0.0'),
                port=self.config.get('webserver_port', 5000)
            )
            self.web_server.start()
            
            logger.info("✓ All systems initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            return False
    
    def start(self):
        """Start main processing loop"""
        if self.running:
            logger.warning("System already running")
            return
        
        self.running = True
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        
        logger.info("Robot system started")
    
    def stop(self):
        """Stop all subsystems"""
        logger.info("Stopping robot system...")
        
        self.running = False
        
        # Stop motors
        if self.safety_controller:
            self.safety_controller.stop()
        
        # Stop camera
        if self.camera:
            self.camera.close()
        
        # Stop SLAM
        if self.slam and hasattr(self.slam, 'stop'):
            self.slam.stop()
        
        # Disconnect motor controller
        if self.motor_controller:
            self.motor_controller.disconnect()
        
        # Save trajectory
        if len(self.trajectory) > 0:
            self._save_trajectory()
        
        logger.info("Robot system stopped")
    
    def _main_loop(self):
        """Main processing loop"""
        logger.info("Main loop started")
        
        while self.running:
            try:
                # 1. Get stereo frames
                frame_data = self.camera.get_frames(timeout=1.0)
                
                if frame_data is None:
                    logger.warning("No frames received")
                    time.sleep(0.1)
                    continue
                
                frame_left, frame_right, timestamp = frame_data
                
                # Store current frames for web streaming
                self.current_frame_left = frame_left
                self.current_frame_right = frame_right
                self.current_timestamp = timestamp
                
                # 2. Rectify frames
                rect_left, rect_right = self.camera.rectify_frames(frame_left, frame_right)
                
                # 3. Run SLAM/Visual Odometry
                if isinstance(self.slam, ORBSLAM3):
                    pose = self.slam.track_stereo(rect_left, rect_right, timestamp)
                else:
                    pose = self.slam.track(rect_left, rect_right)
                
                if pose is not None:
                    self.slam_pose = pose
                    self.slam_tracking_state = "OK"
                    
                    # Log trajectory
                    self.trajectory.append((timestamp, pose.copy()))
                else:
                    self.slam_tracking_state = "LOST"
                
                # 4. Update FPS counter
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = current_time
                    logger.debug(f"FPS: {self.fps:.1f}, Tracking: {self.slam_tracking_state}")
                
            except Exception as e:
                logger.error(f"Main loop error: {e}")
                time.sleep(0.1)
        
        logger.info("Main loop stopped")
    
    def set_velocity(self, linear: float, angular: float) -> bool:
        """Set robot velocity"""
        if self.safety_controller:
            return self.safety_controller.set_velocity(linear, angular)
        return False
    
    def enable_motors(self) -> bool:
        """Enable motors"""
        if self.motor_controller:
            return self.motor_controller.enable()
        return False
    
    def disable_motors(self) -> bool:
        """Disable motors"""
        if self.motor_controller:
            return self.motor_controller.disable()
        return False
    
    def emergency_stop(self) -> bool:
        """Emergency stop"""
        if self.motor_controller:
            return self.motor_controller.stop()
        return False
    
    def get_trajectory(self) -> list:
        """Get SLAM trajectory"""
        return [(t, pose.tolist()) for t, pose in self.trajectory]
    
    def _save_trajectory(self):
        """Save trajectory to file"""
        filename = self.config.get('trajectory_file', 'trajectory.txt')
        
        try:
            with open(filename, 'w') as f:
                f.write("# timestamp tx ty tz qx qy qz qw\n")
                for timestamp, pose in self.trajectory:
                    # Extract translation
                    tx, ty, tz = pose[0, 3], pose[1, 3], pose[2, 3]
                    # For simplicity, use identity quaternion (would need proper conversion)
                    f.write(f"{timestamp} {tx} {ty} {tz} 0 0 0 1\n")
            
            logger.info(f"Trajectory saved to {filename}")
        except Exception as e:
            logger.error(f"Failed to save trajectory: {e}")


def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    logger.info("Interrupt received, shutting down...")
    if 'robot' in globals():
        robot.stop()
    sys.exit(0)


def main():
    """Main entry point"""
    # Configuration
    config = {
        # Serial
        'serial_port': '/dev/ttyACM0',
        'serial_baudrate': 115200,
        
        # Camera
        'camera_left_id': 0,
        'camera_right_id': 1,
        'camera_width': 1280,
        'camera_height': 720,
        'camera_fps': 30,
        'calibration_file': 'calibration.npz',
        
        # ORB-SLAM3
        'use_orbslam': True,
        'orbslam_path': '/home/jetson/ORB_SLAM3',
        'orbslam_vocabulary': '/home/jetson/ORB_SLAM3/Vocabulary/ORBvoc.txt',
        'orbslam_settings': 'orbslam_stereo.yaml',
        'orbslam_viewer': False,  # Set True if you have display
        
        # Robot limits
        'max_linear_speed': 0.5,  # m/s
        'max_angular_speed': 2.0,  # rad/s
        
        # Web server
        'webserver_host': '0.0.0.0',
        'webserver_port': 5000,
        
        # Logging
        'trajectory_file': 'trajectory.txt'
    }
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and initialize robot
    global robot
    robot = RobotSystem(config)
    
    if not robot.initialize():
        logger.error("Failed to initialize robot system")
        return 1
    
    # Start main loop
    robot.start()
    
    logger.info("=" * 60)
    logger.info("Robot system running")
    logger.info(f"Web interface: http://localhost:{config['webserver_port']}")
    logger.info("Press Ctrl+C to stop")
    logger.info("=" * 60)
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
