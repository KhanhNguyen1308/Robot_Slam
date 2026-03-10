#!/usr/bin/env python3
"""
Main Application for Autonomous Mapping Robot
Integrates: Stereo Camera, ORB-SLAM3, Motor Control, Web Server
"""
import sys
import os
import logging
import time
import signal
import threading
import numpy as np
try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import robot modules
from jetson_motor_controller import JetsonMotorController, SafetyController
from stereo_camera import StereoCamera, VisualOdometry
from orb_slam3_wrapper import ORBSLAM3Wrapper, SimpleVisualSLAM
from autonomous_mapper import OccupancyGrid, AutonomousMapper
from obstacle_detector import ObstacleDetector
from mpu6050_imu import MPU6050, ComplementaryFilter
from imu_fusion import IMUVisualFusion, FusedSLAMWrapper
from web_server import init_web_server, run_web_server
from server_client import VisionServerClient
from tts_audio import TTSAudio, Priority as TTSPriority

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/robot.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)


class RobotSystem:
    """
    Main robot system integrating all components
    """
    
    def __init__(self, config: dict):
        self.config = config
        self.running = False
        
        # Components (initialized in setup)
        self.stereo_camera = None
        self.imu_sensor = None
        self.slam_system = None
        self.fused_slam = None
        self.motor_controller = None
        self.obstacle_detector = None
        self.occupancy_grid = None
        self.autonomous_mapper = None

        # 4K object-detection camera + server client
        self.vision_client: VisionServerClient = None

        # Text-to-speech via MAX98357A stereo I2S amplifier
        self.tts: TTSAudio = None

        # Main processing thread
        self.processing_thread = None

        # Watchdog / health monitoring
        self._last_frame_time = 0.0
        self._health_ok = True
        self._watchdog_thread = None

        logger.info("Robot system created")
    
    def setup(self) -> bool:
        """Initialize all robot components"""
        try:
            logger.info("=== Starting Robot System Setup ===")

            # 0. Initialize TTS audio (MAX98357A stereo I2S)
            logger.info("0. Initializing TTS audio (MAX98357A)...")
            self.tts = TTSAudio(self.config)
            self.tts.start()
            self.tts.speak("Robot system starting up.", TTSPriority.HIGH)

            # 1. Initialize Stereo Camera
            logger.info("1. Initializing stereo camera...")
            self.stereo_camera = StereoCamera(
                left_cam_id=self.config['camera']['left_id'],
                right_cam_id=self.config['camera']['right_id'],
                calibration_file=self.config['camera']['calibration_file'],
                stereo_baseline=self.config['camera']['stereo_baseline'],
                width=self.config['camera']['width'],
                height=self.config['camera']['height'],
                fps=self.config['camera']['fps'],
                manual_exposure=self.config['camera'].get('manual_exposure', -6),
                manual_gain=self.config['camera'].get('manual_gain', 50),
                use_manual_settings=self.config['camera'].get('use_manual_settings', True)
            )
            
            if not self.stereo_camera.init_cameras():
                logger.error("Failed to initialize cameras")
                self.tts.speak_critical("Camera initialization failed.")
                return False
            
            # Verify calibration was loaded
            if self.stereo_camera.K_left is None:
                logger.error("Camera calibration failed - cannot proceed")
                return False
            
            # Log baseline being used
            actual_baseline = abs(self.stereo_camera.T[0, 0]) if self.stereo_camera.T is not None else 0.14
            logger.info(f"Stereo baseline in use: {actual_baseline*1000:.1f}mm")
            if abs(actual_baseline - self.config['camera']['stereo_baseline']) > 0.01:
                logger.warning(f"Baseline mismatch! Config: {self.config['camera']['stereo_baseline']*1000:.1f}mm, "
                              f"Actual: {actual_baseline*1000:.1f}mm")
            
            # Start camera capture thread
            self.stereo_camera.start_capture()
            time.sleep(1)  # Let camera warm up
            
            # 2. Initialize IMU (MPU6050)
            logger.info("2. Initializing IMU (MPU6050)...")
            self.imu_sensor = MPU6050(
                bus_number=self.config['imu']['bus_number'],
                address=self.config['imu']['address'],
                gyro_range=self.config['imu']['gyro_range'],
                accel_range=self.config['imu']['accel_range']
            )
            
            if not self.imu_sensor.connect():
                logger.warning("Failed to connect to MPU6050, continuing without IMU")
                self.imu_sensor = None
            else:
                # Calibrate IMU (keep robot stationary!)
                logger.info("Calibrating IMU - keep robot stationary...")
                self.imu_sensor.calibrate(samples=500)
                
                # Start continuous reading
                self.imu_sensor.start_reading()
                logger.info("IMU initialized and calibrated successfully")
            
            # 3. Initialize SLAM System
            logger.info("3. Initializing SLAM system...")
            
            if self.config['slam']['use_orbslam3']:
                # Use ORB-SLAM3 (requires C++ bindings)
                camera_params = {
                    'K_left': self.stereo_camera.K_left,
                    'D_left': self.stereo_camera.D_left,
                    'K_right': self.stereo_camera.K_right,
                    'baseline': abs(self.stereo_camera.T[0][0]),
                    'width': self.config['camera']['width'],
                    'height': self.config['camera']['height'],
                    'fps': self.config['camera']['fps']
                }
                
                self.slam_system = ORBSLAM3Wrapper(
                    orb_slam_path=self.config['slam']['orb_slam_path'],
                    use_viewer=self.config['slam']['use_viewer']
                )
                
                if not self.slam_system.start(camera_params):
                    logger.warning("ORB-SLAM3 failed to start, using simple SLAM")
                    baseline = abs(self.stereo_camera.T[0, 0]) if self.stereo_camera.T is not None else self.config['camera']['stereo_baseline']
                    self.slam_system = SimpleVisualSLAM(self.stereo_camera.K_left, baseline=baseline)
            else:
                # Use simple visual SLAM
                logger.info("Using simple visual SLAM")
                baseline = abs(self.stereo_camera.T[0, 0]) if self.stereo_camera.T is not None else self.config['camera']['stereo_baseline']
                self.slam_system = SimpleVisualSLAM(self.stereo_camera.K_left, baseline=baseline)
            
            # Wrap SLAM with IMU fusion if IMU is available
            if self.imu_sensor is not None:
                logger.info("Wrapping SLAM with IMU sensor fusion...")
                self.fused_slam = FusedSLAMWrapper(
                    slam_system=self.slam_system,
                    imu_sensor=self.imu_sensor,
                    use_ekf=self.config['imu']['use_ekf'],
                    imu_weight=self.config['imu']['fusion_weight']
                )
                self.fused_slam.start()
                logger.info("SLAM-IMU fusion active")
                # Use fused SLAM for all subsequent operations
                slam_for_mapper = self.fused_slam
            else:
                logger.warning("IMU not available, using visual SLAM only")
                slam_for_mapper = self.slam_system
            
            # 4. Initialize Motor Controller
            logger.info("4. Initializing motor controller...")
            base_controller = JetsonMotorController(
                left_step_pin=self.config['motor']['left_step_pin'],
                left_dir_pin=self.config['motor']['left_dir_pin'],
                left_enable_pin=self.config['motor']['left_enable_pin'],
                left_ms1_pin=self.config['motor'].get('left_ms1_pin'),
                left_ms2_pin=self.config['motor'].get('left_ms2_pin'),
                left_ms3_pin=self.config['motor'].get('left_ms3_pin'),
                right_step_pin=self.config['motor']['right_step_pin'],
                right_dir_pin=self.config['motor']['right_dir_pin'],
                right_enable_pin=self.config['motor']['right_enable_pin'],
                right_ms1_pin=self.config['motor'].get('right_ms1_pin'),
                right_ms2_pin=self.config['motor'].get('right_ms2_pin'),
                right_ms3_pin=self.config['motor'].get('right_ms3_pin'),
                wheel_diameter=self.config['motor']['wheel_diameter'],
                wheel_base=self.config['motor']['wheel_base'],
                steps_per_rev=self.config['motor']['steps_per_rev'],
                microsteps=self.config['motor']['microsteps'],
                gear_ratio=self.config['motor'].get('gear_ratio')
            )
            
            # Wrap with safety controller
            self.motor_controller = SafetyController(
                base_controller,
                max_linear=self.config['motor']['max_linear_speed'],
                max_angular=self.config['motor']['max_angular_speed']
            )
            
            if not self.motor_controller.connect():
                logger.error("Failed to initialize motor controller")
                self.tts.speak_critical("Motor controller initialization failed.")
                return False
            
            # 5. Initialize Obstacle Detector
            logger.info("5. Initializing obstacle detector...")
            self.obstacle_detector = ObstacleDetector(
                min_safe_distance=self.config.get('obstacle_detection', {}).get('min_safe_distance', 0.3),
                critical_distance=self.config.get('obstacle_detection', {}).get('critical_distance', 0.15),
                detection_width=self.config.get('obstacle_detection', {}).get('detection_width', 0.5),
                max_detection_range=self.config.get('obstacle_detection', {}).get('max_range', 2.0)
            )
            logger.info(f"Obstacle detector ready: safe_dist={self.obstacle_detector.min_safe_distance}m")
            
            # 6. Create Occupancy Grid
            logger.info("6. Creating occupancy grid...")
            self.occupancy_grid = OccupancyGrid(
                width=self.config['mapping']['grid_width'],
                height=self.config['mapping']['grid_height'],
                resolution=self.config['mapping']['resolution']
            )
            
            # 7. Create Autonomous Mapper
            logger.info("7. Initializing autonomous mapper...")
            _mapper_baseline = (
                abs(self.stereo_camera.T[0, 0])
                if self.stereo_camera.T is not None
                else self.config['camera']['stereo_baseline']
            )
            self.autonomous_mapper = AutonomousMapper(
                motor_controller=self.motor_controller,
                slam_system=slam_for_mapper,  # Use fused SLAM if available
                occupancy_grid=self.occupancy_grid,
                obstacle_detector=self.obstacle_detector,
                max_linear_speed=self.config['motor']['max_linear_speed'],
                max_angular_speed=self.config['motor']['max_angular_speed'],
                camera_matrix=self.stereo_camera.K_left,
                baseline=_mapper_baseline,
                robot_radius=self.config.get('robot', {}).get('radius', 0.2)
            )
            
            # 8. Initialize UGREEN 4K camera + vision server client
            vs_cfg = self.config.get('vision_server', {})
            oc_cfg = self.config.get('object_camera', {})
            if vs_cfg.get('enabled', False):
                logger.info("8. Initializing vision server client (YOLOv11)...")
                self.vision_client = VisionServerClient(
                    device_id=oc_cfg.get('device_id', 2),
                    capture_width=oc_cfg.get('capture_width', 3840),
                    capture_height=oc_cfg.get('capture_height', 2160),
                    capture_fps=oc_cfg.get('capture_fps', 15),
                    stream_width=oc_cfg.get('stream_width', 1280),
                    stream_height=oc_cfg.get('stream_height', 720),
                    jpeg_quality=oc_cfg.get('jpeg_quality', 85),
                    server_host=vs_cfg.get('host', '192.168.2.10'),
                    server_port=vs_cfg.get('port', 8090),
                    timeout_s=vs_cfg.get('timeout_s', 0.15),
                    min_confidence=vs_cfg.get('min_confidence', 0.40),
                )
                if self.vision_client.start():
                    # Give mapper access to horizontal FOV for bearing calc
                    self.autonomous_mapper.vision_client = self.vision_client
                    self.autonomous_mapper._camera_hfov_deg = vs_cfg.get(
                        'camera_hfov_deg', 90.0
                    )
                    logger.info("Vision server client started (UGREEN 4K → x99 YOLOv11)")
                else:
                    logger.warning(
                        "Vision server client failed to open camera — "
                        "object detection disabled"
                    )
                    self.vision_client = None
            else:
                logger.info("8. Vision server disabled (vision_server.enabled=false)")

            logger.info("=== Robot System Setup Complete ===")
            self.tts.speak("All systems ready. Beginning autonomous mapping.",
                           TTSPriority.HIGH)
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}", exc_info=True)
            return False
    
    def start(self):
        """Start main processing loop"""
        self.running = True
        self.processing_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.processing_thread.start()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="watchdog"
        )
        self._watchdog_thread.start()
        logger.info("Main processing loop started")
    
    def _main_loop(self):
        """Main processing loop - integrates SLAM, mapping, and obstacle detection"""
        logger.info("Entering main processing loop")
        
        frame_count = 0
        start_time = time.time()
        
        while self.running:
            try:
                # Get latest frames from camera
                left, right, disparity = self.stereo_camera.get_latest_frames()
                
                if left is not None and right is not None:
                    # Process through SLAM
                    timestamp = time.time()
                    
                    if isinstance(self.slam_system, ORBSLAM3Wrapper):
                        # ORB-SLAM3
                        pose = self.slam_system.process_frame(left, right, timestamp)
                    else:
                        # Simple SLAM
                        pose = self.slam_system.track(left, right)
                    
                    # Update obstacle detection from disparity
                    if disparity is not None and self.obstacle_detector:
                        # Get camera parameters
                        camera_matrix = self.stereo_camera.K_left
                        baseline = abs(self.stereo_camera.T[0, 0]) if self.stereo_camera.T is not None else self.config['camera']['stereo_baseline']
                        
                        # Run obstacle detection
                        obstacle_result = self.autonomous_mapper.update_obstacle_detection(
                            disparity, camera_matrix, baseline
                        )
                        
                        # Log critical obstacles
                        if obstacle_result and obstacle_result['action']['severity'] == 'critical':
                            logger.warning(f"CRITICAL OBSTACLE: {obstacle_result['action']['message']}")
                            self.tts.speak_critical("Obstacle detected. Stopping.")
                    
                    # Update mapper with disparity map
                    if disparity is not None and pose is not None:
                        self.autonomous_mapper.update_map_from_stereo(disparity, pose)

                    # Feed YOLO detections into semantic map (non-blocking)
                    if self.vision_client is not None:
                        detections = self.vision_client.get_latest_detections()
                        if detections:
                            self.autonomous_mapper.update_semantic_detections(detections)

                    frame_count += 1
                    self._last_frame_time = time.time()

                    # Log statistics every 100 frames
                    if frame_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = frame_count / elapsed
                        logger.info(f"Processed {frame_count} frames at {fps:.1f} FPS")
                        
                        # Print SLAM stats
                        slam_stats = self.slam_system.get_stats()
                        logger.info(f"SLAM: {slam_stats}")
                        
                        # Print mapper stats
                        mapper_stats = self.autonomous_mapper.get_stats()
                        logger.info(f"Mapper: mode={mapper_stats['mode']}, "
                                  f"coverage={mapper_stats['coverage_percent']:.1f}%, "
                                  f"obstacles_avoided={mapper_stats.get('obstacles_avoided', 0)}")

                        # Semantic map summary
                        sem = mapper_stats.get('semantic_objects', {})
                        if sem:
                            summary = ', '.join(f"{k}:{v}" for k, v in sem.items())
                            logger.info(f"Semantic map: {summary}")
                        if self.vision_client:
                            logger.info(f"Vision client: {self.vision_client.get_stats()}")

                        # Announce mapping coverage milestones
                        if frame_count % 100 == 0 and self.autonomous_mapper:
                            stats = self.autonomous_mapper.get_stats()
                            coverage = stats.get('coverage_percent', 0.0)
                            for milestone in (25, 50, 75, 90):
                                attr = f'_announced_{milestone}pct'
                                if coverage >= milestone and not getattr(self, attr, False):
                                    setattr(self, attr, True)
                                    self.tts.speak_event(
                                        f"Mapping {milestone} percent complete."
                                    )

                time.sleep(0.01)  # ~100 Hz max
                
            except Exception as e:
                logger.error(f"Main loop error: {e}", exc_info=True)
                time.sleep(1.0)

    def _watchdog_loop(self):
        """Monitor the main processing loop; stop motors if it goes silent."""
        WATCHDOG_TIMEOUT = 5.0  # seconds without a frame before declaring unhealthy
        while self.running:
            time.sleep(1.0)
            if self._last_frame_time > 0:
                age = time.time() - self._last_frame_time
                if age > WATCHDOG_TIMEOUT:
                    if self._health_ok:
                        self._health_ok = False
                        logger.critical(
                            f"Watchdog: main loop stalled for {age:.1f}s — stopping motors"
                        )
                        if self.motor_controller:
                            try:
                                self.motor_controller.stop()
                            except Exception as exc:
                                logger.error(f"Watchdog motor-stop failed: {exc}")
                else:
                    if not self._health_ok:
                        logger.info("Watchdog: main loop recovered")
                    self._health_ok = True

    def stop(self):
        """Stop all robot systems"""
        logger.info("Stopping robot system...")
        if self.tts:
            self.tts.speak("Robot shutting down.", TTSPriority.HIGH)
            # Brief pause so the utterance can finish before motors stop
            time.sleep(1.5)

        self.running = False
        
        # Stop autonomous exploration
        if self.autonomous_mapper:
            self.autonomous_mapper.stop_exploration()
        
        # Stop motors
        if self.motor_controller:
            self.motor_controller.stop()
            time.sleep(0.5)
            self.motor_controller.disable()
        
        # Stop camera
        if self.stereo_camera:
            self.stereo_camera.stop_capture()
            self.stereo_camera.release()

        # Stop vision server client
        if self.vision_client:
            self.vision_client.stop()

        # Stop IMU
        if self.imu_sensor:
            self.imu_sensor.disconnect()
        
        # Stop fused SLAM
        if self.fused_slam:
            self.fused_slam.stop()
        
        # Stop SLAM
        if self.slam_system:
            if hasattr(self.slam_system, 'shutdown'):
                self.slam_system.shutdown()
        
        # Wait for processing thread
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)

        # Stop TTS last so any final messages can complete
        if self.tts:
            self.tts.stop()

        logger.info("Robot system stopped")
    
    def cleanup(self):
        """Final cleanup"""
        if self.motor_controller:
            self.motor_controller.disconnect()
        logger.info("Cleanup complete")


def load_config():
    """
    Load configuration from config.yaml if present, otherwise use hardcoded defaults.
    Edit config.yaml to change hardware settings without touching Python source.
    """
    yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    if _YAML_AVAILABLE and os.path.exists(yaml_path):
        try:
            with open(yaml_path, "r") as f:
                cfg = yaml.safe_load(f)
            logger.info(f"Configuration loaded from {yaml_path}")
            return cfg
        except Exception as e:
            logger.warning(f"Failed to read config.yaml ({e}), using defaults")

    logger.warning("config.yaml not found or yaml unavailable — using hardcoded defaults")
    # --- hardcoded defaults (kept as fallback) ---
    config = {
        'camera': {
            'left_id': 2,
            'right_id': 1,
            'calibration_file': 'calibration.npz',
            'stereo_baseline': 0.14,  # 14cm baseline - IMPORTANT: must match hardware!
            'width': 640,
            'height': 480,
            'fps': 30,
            # Manual camera settings to fix auto-exposure/white-balance mismatch
            'use_manual_settings': True,  # CRITICAL: Set to True to fix color mismatch
            'manual_exposure': -5,        # Increased from -6 for brighter image
            'manual_gain': 55             # Increased from 50 for better brightness match
        },
        'slam': {
            'use_orbslam3': False,  # Set to True if ORB-SLAM3 is properly built
            'orb_slam_path': '/home/jetson/ORB_SLAM3',
            'use_viewer': False
        },
        'motor': {
            'left_step_pin': 33,
            'left_dir_pin': 35,
            'left_enable_pin': 37,
            'left_ms1_pin': 29,  # Set if using microstepping control
            'left_ms2_pin': 31,
            'left_ms3_pin': 11,
            'right_step_pin': 32,
            'right_dir_pin': 36,
            'right_enable_pin': 38,
            'right_ms1_pin': 13,
            'right_ms2_pin': 15,
            'right_ms3_pin': 16,
            
            # Track/Sprocket configuration (Updated 2026-02-22)
            # Custom sprocket: 20 teeth (mounted on intermediate shaft with 38-tooth gear)
            # Custom design from CAD:
            #   - Outer diameter: 69.84mm (R=34.9218mm) ← ACTUAL working diameter
            #   - Inner diameter: 60mm (R=30mm)
            # Track separation: 284.5mm (center to center)
            'wheel_diameter': 0.06984,  # 69.84mm outer diameter from CAD (confirmed)
            'wheel_base': 0.2845,       # 284.5mm track separation
            'steps_per_rev': 200,
            'microsteps': 16,
            
            # Gear ratio configuration:
            # Motor shaft: 18-tooth gear
            # Intermediate shaft: 38-tooth gear (receives from motor) + 20-tooth sprocket (same shaft)
            # Single-stage reduction: motor must rotate 38/18 times for sprocket to rotate once
            'gear_ratio': 38/18,  # 2.111 (motor rotations per wheel rotation)
            
            'max_linear_speed': 0.3,   # m/s
            'max_angular_speed': 1.5   # rad/s
        },
        'mapping': {
            'grid_width': 10.0,    # meters
            'grid_height': 10.0,   # meters
            'resolution': 0.05     # meters per cell
        },
        'obstacle_detection': {
            'min_safe_distance': 0.3,    # meters - minimum safe distance
            'critical_distance': 0.15,   # meters - emergency stop distance
            'detection_width': 0.5,      # meters - robot width + safety margin
            'max_range': 2.0             # meters - maximum detection range
        },
        'imu': {
            'bus_number': 1,              # I2C bus 1 on Jetson Nano
            'address': 0x68,              # Default MPU6050 address
            'gyro_range': 250,            # deg/s (250, 500, 1000, 2000)
            'accel_range': 2,             # g (2, 4, 8, 16)
            'use_ekf': True,              # Use Extended Kalman Filter for fusion
            'fusion_weight': 0.3          # IMU weight in complementary filter (if not using EKF)
        },
        'web_server': {
            'host': '0.0.0.0',
            'port': 5000,
            'debug': False
        }
    }
    
    return config


def signal_handler(signum, frame):
    """Fallback signal handler (overridden in main() with a proper closure)."""
    sys.exit(0)


def main():
    """Main application entry point"""
    logger.info("=" * 60)
    logger.info("Autonomous Mapping Robot - Starting")
    logger.info("=" * 60)
    
    # Load configuration
    config = load_config()
    logger.info("Configuration loaded")

    # Create robot system
    robot = RobotSystem(config)

    # Register signal handler now that robot exists (closure — no globals() needed)
    def _shutdown_handler(signum, frame):
        logger.info("Shutdown signal received")
        robot.stop()
        robot.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)
    
    # Setup all components
    if not robot.setup():
        logger.error("Robot setup failed, exiting")
        return 1
    
    # Enable motors
    logger.info("Enabling motors...")
    robot.motor_controller.enable()
    
    # Start main processing
    robot.start()
    
    # Initialize web server
    logger.info("Initializing web server...")
    init_web_server(
        robot.motor_controller,
        robot.fused_slam if robot.fused_slam else robot.slam_system,
        robot.stereo_camera,
        robot.autonomous_mapper,
        robot.imu_sensor,
        robot.vision_client,
    )
    
    # Start web server (blocking)
    logger.info(f"Starting web server on {config['web_server']['host']}:{config['web_server']['port']}")
    logger.info("Access dashboard at: http://<jetson-ip>:5000")
    
    try:
        run_web_server(
            host=config['web_server']['host'],
            port=config['web_server']['port'],
            debug=config['web_server']['debug']
        )
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        robot.stop()
        robot.cleanup()
    
    logger.info("Application terminated")
    return 0


if __name__ == "__main__":
    sys.exit(main())