"""
Main Robot Controller
Integrates all subsystems: Camera, SLAM, Motor Control, Web Server, IMU, Obstacle Detection
"""
import numpy as np
import cv2
import time
import logging
import threading
import signal
import sys
from typing import Optional

from jetson_motor_controller import JetsonMotorController, SafetyController
from stereo_camera import StereoCamera
from orbslam_interface import ORBSLAM3, SimpleVisualOdometry
from web_server import RobotWebServer
from imu_module import ADXL345, IMUFilter
from obstacle_detection import ObstacleDetector, SimpleObstacleAvoidance
from library_mapper import LibraryMapper, LibraryMapVisualizer

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
        self.imu = None
        self.imu_filter = None
        self.obstacle_detector = None
        self.obstacle_avoidance = None
        self.library_mapper = None
        self.map_visualizer = None
        
        # State
        self.running = False
        self.slam_pose = np.eye(4)
        self.slam_tracking_state = "NOT_INITIALIZED"
        self.tracking_lost_count = 0
        self.last_tracking_log_time = 0
        
        # Current frames
        self.current_frame_left = None
        self.current_frame_right = None
        self.current_timestamp = 0
        
        # Depth and obstacles
        self.current_depth_map = None
        self.current_obstacles = []
        
        # IMU data
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.collision_detected = False
        
        # Trajectory
        self.trajectory = []
        
        # Performance metrics
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Control mode
        self.autonomous_mode = False
        self.mapping_mode = False
        
        # Main processing thread
        self.main_thread = None
        
    def initialize(self) -> bool:
        """Initialize all subsystems"""
        logger.info("Initializing robot system...")
        
        try:
            # 1. Initialize motor controller (GPIO-based)
            logger.info("Initializing Jetson GPIO motor controller...")
            self.motor_controller = JetsonMotorController(
                left_step_pin=self.config.get('left_step_pin', 33),
                left_dir_pin=self.config.get('left_dir_pin', 35),
                left_enable_pin=self.config.get('left_enable_pin', 37),
                left_ms1_pin=self.config.get('left_ms1_pin'),
                left_ms2_pin=self.config.get('left_ms2_pin'),
                left_ms3_pin=self.config.get('left_ms3_pin'),
                right_step_pin=self.config.get('right_step_pin', 32),
                right_dir_pin=self.config.get('right_dir_pin', 36),
                right_enable_pin=self.config.get('right_enable_pin', 38),
                right_ms1_pin=self.config.get('right_ms1_pin'),
                right_ms2_pin=self.config.get('right_ms2_pin'),
                right_ms3_pin=self.config.get('right_ms3_pin'),
                wheel_diameter=self.config.get('wheel_diameter', 0.066),
                wheel_base=self.config.get('wheel_base', 0.165),
                steps_per_rev=self.config.get('steps_per_rev', 200),
                microsteps=self.config.get('microsteps', 16)
            )
            
            if not self.motor_controller.connect():
                logger.error("Failed to initialize motor controller")
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
                left_id=self.config.get('camera_left_id', 1),
                right_id=self.config.get('camera_right_id', 0),
                calibration_file=self.config.get('calibration_file', 'calibration.npz'),
                width=self.config.get('camera_width', 1280),
                height=self.config.get('camera_height', 720),
                fps=self.config.get('camera_fps', 30)
            )
            
            if not self.camera.open():
                logger.error("Failed to open cameras")
                return False
            
            self.camera.start_capture()
            
            # 4. Initialize IMU (optional)
            use_imu = self.config.get('use_imu', True)
            
            if use_imu:
                logger.info("Initializing IMU ADXL345...")
                self.imu = ADXL345(bus=1, g_range=2)
                
                if self.imu.connect():
                    # Calibrate IMU
                    logger.info("Calibrating IMU (keep robot still)...")
                    time.sleep(1)
                    self.imu.calibrate()
                    
                    # Start filtering
                    self.imu_filter = IMUFilter(self.imu, alpha=0.98, update_rate=50)
                    self.imu_filter.start()
                    
                    logger.info("✓ IMU initialized")
                else:
                    logger.warning("IMU not available - continuing without it")
                    self.imu = None
            
            # 5. Initialize obstacle detection
            logger.info("Initializing obstacle detection...")
            
            if self.camera.calibration_loaded:
                # Get camera parameters
                baseline = abs(self.camera.T[0])  # Baseline in meters
                focal_length = self.camera.K1[0, 0]  # Focal length in pixels
                
                self.obstacle_detector = ObstacleDetector(
                    camera_baseline=baseline,
                    camera_focal_length=focal_length,
                    min_distance=0.2,
                    max_distance=3.0,
                    danger_zone_distance=self.config.get('danger_zone_distance', 0.5)
                )
                
                self.obstacle_avoidance = SimpleObstacleAvoidance(self.obstacle_detector)
                
                logger.info("✓ Obstacle detection initialized")
            else:
                logger.warning("Cannot initialize obstacle detection without calibration")
            
            # 6. Initialize library mapper
            logger.info("Initializing library mapper...")
            
            self.library_mapper = LibraryMapper(
                map_width=self.config.get('map_width', 15.0),
                map_height=self.config.get('map_height', 15.0),
                resolution=self.config.get('map_resolution', 0.05)
            )
            
            # Create visualizer if requested
            if self.config.get('show_map_viz', False):
                self.map_visualizer = LibraryMapVisualizer(self.library_mapper)
            
            logger.info("✓ Library mapper initialized")
            
            # 7. Initialize ORB-SLAM3 (optional)
            use_orbslam = self.config.get('use_orbslam', True)
            
            if use_orbslam and self.camera.calibration_loaded:
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
                if not self.camera.calibration_loaded:
                    logger.warning("Cannot initialize ORB-SLAM3 without calibration - using simple visual odometry")
                else:
                    logger.info("Using simple visual odometry")
                self.slam = SimpleVisualOdometry()
            
            # 8. Initialize web server
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
            import traceback
            logger.error(f"Initialization failed: {e}")
            logger.error(traceback.format_exc())
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
        
        # Disable autonomous mode
        self.autonomous_mode = False
        
        # Stop motors
        if self.safety_controller:
            self.safety_controller.stop()
        
        # Stop camera
        if self.camera:
            self.camera.close()
        
        # Stop IMU
        if self.imu_filter:
            self.imu_filter.stop()
        if self.imu:
            self.imu.close()
        
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
                
                # 3. Obstacle detection
                if self.obstacle_detector:
                    # Compute disparity and depth
                    disparity = self.obstacle_detector.compute_disparity(rect_left, rect_right)
                    depth_map = self.obstacle_detector.disparity_to_depth(disparity)
                    self.current_depth_map = depth_map
                    
                    # Detect obstacles
                    obstacles = self.obstacle_detector.detect_obstacles(depth_map, rect_left)
                    self.current_obstacles = obstacles
                    
                    # Mapping mode - use depth for occupancy grid
                    if self.mapping_mode and self.library_mapper:
                        camera_params = {
                            'focal_length': self.camera.K1[0, 0],
                            'width': self.camera.width,
                            'height': self.camera.height
                        }
                        
                        # Update mapper
                        linear, angular = self.library_mapper.update(
                            self.slam_pose,
                            depth_map,
                            camera_params,
                            obstacles
                        )
                        
                        # Scale velocity based on tracking quality
                        linear, angular = self._scale_velocity_by_tracking(linear, angular)
                        
                        # Send velocity commands
                        self.set_velocity(linear, angular)
                        
                        # Update visualization if enabled
                        if self.map_visualizer:
                            self.map_visualizer.show(self.slam_pose)
                    
                    # Autonomous obstacle avoidance (only if not mapping)
                    elif self.autonomous_mode and self.obstacle_avoidance:
                        linear, angular = self.obstacle_avoidance.compute_velocity(obstacles)
                        
                        # Scale velocity based on tracking quality
                        linear, angular = self._scale_velocity_by_tracking(linear, angular)
                        
                        self.set_velocity(linear, angular)
                
                # 4. Update IMU
                if self.imu_filter:
                    self.imu_pitch, self.imu_roll = self.imu_filter.get_orientation()
                    
                    # Check for collision
                    if self.imu_filter.detect_collision(threshold=2.5):
                        if not self.collision_detected:
                            logger.warning("⚠ COLLISION DETECTED!")
                            self.collision_detected = True
                            self.emergency_stop()
                    else:
                        self.collision_detected = False
                    
                    # Check for excessive tilt
                    if self.imu_filter.is_tilted(max_angle=20.0):
                        logger.warning("⚠ Robot tilted beyond safe angle!")
                
                # 5. Run SLAM/Visual Odometry
                if isinstance(self.slam, ORBSLAM3):
                    pose = self.slam.track_stereo(rect_left, rect_right, timestamp)
                else:
                    pose = self.slam.track(rect_left, rect_right)
                
                if pose is not None:
                    self.slam_pose = pose
                    prev_state = self.slam_tracking_state
                    
                    # Determine tracking state based on quality
                    if hasattr(self.slam, 'tracking_quality'):
                        quality = self.slam.tracking_quality
                        if quality > 0.5:
                            self.slam_tracking_state = "OK"
                        elif quality > 0.2:
                            self.slam_tracking_state = "DEGRADED"
                        else:
                            self.slam_tracking_state = "POOR"
                    else:
                        self.slam_tracking_state = "OK"
                    
                    # Log when tracking is recovered
                    if prev_state == "LOST" and self.tracking_lost_count > 0:
                        quality_info = ""
                        if hasattr(self.slam, 'tracking_quality'):
                            quality_info = f", quality: {self.slam.tracking_quality:.2f}"
                        logger.info(f"✓ Tracking recovered (was lost for {self.tracking_lost_count} frames{quality_info})")
                        self.tracking_lost_count = 0
                    
                    # Slow down robot if tracking quality is poor
                    if self.slam_tracking_state in ["DEGRADED", "POOR"]:
                        if self.autonomous_mode or self.mapping_mode:
                            # Reduce speed when tracking is poor
                            if hasattr(self, '_speed_reduction_logged'):
                                if not self._speed_reduction_logged:
                                    logger.warning(f"⚠ Tracking quality poor ({quality:.2f}) - reducing speed")
                                    self._speed_reduction_logged = True
                            else:
                                self._speed_reduction_logged = True
                                logger.warning(f"⚠ Tracking quality poor ({quality:.2f}) - reducing speed")
                    else:
                        self._speed_reduction_logged = False
                    
                    # Log trajectory
                    self.trajectory.append((timestamp, pose.copy()))
                else:
                    prev_state = self.slam_tracking_state
                    self.slam_tracking_state = "LOST"
                    self.tracking_lost_count += 1
                    
                    # Stop or slow down robot when tracking is lost
                    if self.tracking_lost_count > 5 and (self.autonomous_mode or self.mapping_mode):
                        logger.warning("⚠ Stopping robot - tracking lost for too long")
                        self.emergency_stop()
                        # Give system time to recover
                        time.sleep(0.5)
                    
                    # Log when tracking is lost (throttled to every 5 seconds)
                    current_time = time.time()
                    if prev_state != "LOST":
                        reason = "insufficient features"
                        if hasattr(self.slam, 'frames_since_init'):
                            reason += f" (frame {self.slam.frames_since_init})"
                        if hasattr(self.slam, 'consecutive_failures'):
                            reason += f", consecutive failures: {self.slam.consecutive_failures}"
                        logger.warning(f"⚠ Tracking lost - {reason}")
                        self.last_tracking_log_time = current_time
                    elif current_time - self.last_tracking_log_time > 5.0:
                        logger.warning(f"⚠ Still lost tracking ({self.tracking_lost_count} frames)")
                        self.last_tracking_log_time = current_time
                
                # 6. Update FPS counter
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = current_time
                    
                    # Log status
                    logger.debug(f"FPS: {self.fps:.1f}, "
                               f"SLAM: {self.slam_tracking_state}, "
                               f"Obstacles: {len(self.current_obstacles)}, "
                               f"IMU: P={self.imu_pitch:.1f}° R={self.imu_roll:.1f}°")
                
            except Exception as e:
                logger.error(f"Main loop error: {e}")
                time.sleep(0.1)
        
        logger.info("Main loop stopped")
    
    def _scale_velocity_by_tracking(self, linear: float, angular: float) -> tuple:
        """
        Scale velocity commands based on tracking quality
        Reduces speed when tracking is poor to improve feature detection
        
        Returns:
            (scaled_linear, scaled_angular)
        """
        if self.slam_tracking_state == "OK":
            return linear, angular
        elif self.slam_tracking_state == "DEGRADED":
            # Reduce to 50% speed
            scale_factor = 0.5
            return linear * scale_factor, angular * scale_factor
        elif self.slam_tracking_state == "POOR":
            # Reduce to 25% speed
            scale_factor = 0.25
            return linear * scale_factor, angular * scale_factor
        elif self.slam_tracking_state == "LOST":
            # Stop completely
            return 0.0, 0.0
        else:
            return linear, angular
    
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
    
    def enable_autonomous_mode(self, enable: bool = True):
        """Enable/disable autonomous obstacle avoidance"""
        self.autonomous_mode = enable
        if enable:
            self.mapping_mode = False  # Disable mapping when autonomous mode enabled
        logger.info(f"Autonomous mode: {'ENABLED' if enable else 'DISABLED'}")
    
    def enable_mapping_mode(self, enable: bool = True):
        """Enable/disable autonomous mapping"""
        self.mapping_mode = enable
        
        if enable:
            self.autonomous_mode = False  # Disable autonomous when mapping enabled
            if self.library_mapper:
                self.library_mapper.start_mapping()
                logger.info("🗺️ MAPPING MODE ENABLED - Starting autonomous exploration")
        else:
            if self.library_mapper:
                self.library_mapper.stop_mapping()
                logger.info("Mapping mode disabled")
    
    def save_current_map(self, filename: str = None):
        """Save current map"""
        if not self.library_mapper:
            logger.warning("Library mapper not initialized")
            return False
        
        if filename is None:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"library_map_{timestamp}"
        
        self.library_mapper.save_map(filename)
        return True
    
    def get_mapping_status(self) -> dict:
        """Get mapping status"""
        if not self.library_mapper:
            return {
                "mapping_enabled": False,
                "mapper_available": False
            }
        
        status = self.library_mapper.get_status()
        status['mapping_enabled'] = self.mapping_mode
        status['mapper_available'] = True
        
        return status
    
    def get_obstacle_info(self) -> dict:
        """Get current obstacle information"""
        if not self.obstacle_detector or not self.current_obstacles:
            return {
                "obstacles": [],
                "danger_zones": {"left": "clear", "center": "clear", "right": "clear"},
                "safe_direction": "forward"
            }
        
        zones = self.obstacle_detector.get_danger_zones(self.current_obstacles)
        direction, confidence = self.obstacle_detector.get_safe_direction(self.current_obstacles)
        
        obstacles_data = [{
            "distance": float(obs.distance),
            "angle": float(obs.angle),
            "severity": obs.severity
        } for obs in self.current_obstacles[:5]]  # Top 5 closest
        
        return {
            "obstacles": obstacles_data,
            "danger_zones": zones,
            "safe_direction": direction,
            "confidence": float(confidence)
        }
    
    def get_imu_data(self) -> dict:
        """Get IMU sensor data"""
        if not self.imu:
            return {
                "connected": False,
                "pitch": 0.0,
                "roll": 0.0,
                "collision": False
            }
        
        ax, ay, az = self.imu.read_accel()
        
        return {
            "connected": True,
            "pitch": float(self.imu_pitch),
            "roll": float(self.imu_roll),
            "accel_x": float(ax),
            "accel_y": float(ay),
            "accel_z": float(az),
            "magnitude": float(self.imu.get_magnitude()),
            "collision": bool(self.collision_detected)
        }
    
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
        
        # IMU
        'use_imu': True,
        'imu_bus': 1,
        'imu_address': 0x53,
        
        # Obstacle Detection
        'danger_zone_distance': 0.5,  # meters
        
        # Library Mapping
        'map_width': 15.0,  # meters
        'map_height': 15.0,  # meters
        'map_resolution': 0.05,  # meters per cell (5cm)
        'show_map_viz': False,  # Set True to show live map window
        
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