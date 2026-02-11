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

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import robot modules
from jetson_motor_controller import JetsonMotorController, SafetyController
from stereo_camera import StereoCamera, VisualOdometry
from orb_slam3_wrapper import ORBSLAM3Wrapper, SimpleVisualSLAM
from autonomous_mapper import OccupancyGrid, AutonomousMapper
from web_server import init_web_server, run_web_server

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot.log'),
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
        self.slam_system = None
        self.motor_controller = None
        self.occupancy_grid = None
        self.autonomous_mapper = None
        
        # Main processing thread
        self.processing_thread = None
        
        logger.info("Robot system created")
    
    def setup(self) -> bool:
        """Initialize all robot components"""
        try:
            logger.info("=== Starting Robot System Setup ===")
            
            # 1. Initialize Stereo Camera
            logger.info("1. Initializing stereo camera...")
            self.stereo_camera = StereoCamera(
                left_cam_id=self.config['camera']['left_id'],
                right_cam_id=self.config['camera']['right_id'],
                calibration_file=self.config['camera']['calibration_file'],
                width=self.config['camera']['width'],
                height=self.config['camera']['height'],
                fps=self.config['camera']['fps']
            )
            
            if not self.stereo_camera.init_cameras():
                logger.error("Failed to initialize cameras")
                return False
            
            # Verify calibration was loaded
            if self.stereo_camera.K_left is None:
                logger.error("Camera calibration failed - cannot proceed")
                return False
            
            # Start camera capture thread
            self.stereo_camera.start_capture()
            time.sleep(1)  # Let camera warm up
            
            # 2. Initialize SLAM System
            logger.info("2. Initializing SLAM system...")
            
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
                    self.slam_system = SimpleVisualSLAM(self.stereo_camera.K_left)
            else:
                # Use simple visual SLAM
                logger.info("Using simple visual SLAM")
                self.slam_system = SimpleVisualSLAM(self.stereo_camera.K_left)
            
            # 3. Initialize Motor Controller
            logger.info("3. Initializing motor controller...")
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
                microsteps=self.config['motor']['microsteps']
            )
            
            # Wrap with safety controller
            self.motor_controller = SafetyController(
                base_controller,
                max_linear=self.config['motor']['max_linear_speed'],
                max_angular=self.config['motor']['max_angular_speed']
            )
            
            if not self.motor_controller.connect():
                logger.error("Failed to initialize motor controller")
                return False
            
            # 4. Create Occupancy Grid
            logger.info("4. Creating occupancy grid...")
            self.occupancy_grid = OccupancyGrid(
                width=self.config['mapping']['grid_width'],
                height=self.config['mapping']['grid_height'],
                resolution=self.config['mapping']['resolution']
            )
            
            # 5. Create Autonomous Mapper
            logger.info("5. Initializing autonomous mapper...")
            self.autonomous_mapper = AutonomousMapper(
                motor_controller=self.motor_controller,
                slam_system=self.slam_system,
                occupancy_grid=self.occupancy_grid,
                max_linear_speed=self.config['motor']['max_linear_speed'],
                max_angular_speed=self.config['motor']['max_angular_speed']
            )
            
            logger.info("=== Robot System Setup Complete ===")
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}", exc_info=True)
            return False
    
    def start(self):
        """Start main processing loop"""
        self.running = True
        self.processing_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.processing_thread.start()
        logger.info("Main processing loop started")
    
    def _main_loop(self):
        """Main processing loop - integrates SLAM and mapping"""
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
                    
                    # Update mapper with disparity map
                    if disparity is not None and pose is not None:
                        self.autonomous_mapper.update_map_from_stereo(disparity, pose)
                    
                    frame_count += 1
                    
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
                                  f"coverage={mapper_stats['coverage_percent']:.1f}%")
                
                time.sleep(0.01)  # ~100 Hz max
                
            except Exception as e:
                logger.error(f"Main loop error: {e}", exc_info=True)
                time.sleep(1.0)
    
    def stop(self):
        """Stop all robot systems"""
        logger.info("Stopping robot system...")
        
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
        
        # Stop SLAM
        if self.slam_system:
            if hasattr(self.slam_system, 'shutdown'):
                self.slam_system.shutdown()
        
        # Wait for processing thread
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        
        logger.info("Robot system stopped")
    
    def cleanup(self):
        """Final cleanup"""
        if self.motor_controller:
            self.motor_controller.disconnect()
        logger.info("Cleanup complete")


def load_config():
    """Load configuration from file or use defaults"""
    config = {
        'camera': {
            'left_id': 0,
            'right_id': 1,
            'calibration_file': 'calibration.npz',
            'width': 640,
            'height': 480,
            'fps': 30
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
            'left_ms1_pin': None,  # Set if using microstepping control
            'left_ms2_pin': None,
            'left_ms3_pin': None,
            'right_step_pin': 32,
            'right_dir_pin': 36,
            'right_enable_pin': 38,
            'right_ms1_pin': None,
            'right_ms2_pin': None,
            'right_ms3_pin': None,
            'wheel_diameter': 0.066,  # 66mm
            'wheel_base': 0.165,      # 165mm
            'steps_per_rev': 200,
            'microsteps': 16,
            'max_linear_speed': 0.3,   # m/s
            'max_angular_speed': 1.5   # rad/s
        },
        'mapping': {
            'grid_width': 10.0,    # meters
            'grid_height': 10.0,   # meters
            'resolution': 0.05     # meters per cell
        },
        'web_server': {
            'host': '0.0.0.0',
            'port': 5000,
            'debug': False
        }
    }
    
    return config


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    logger.info("Shutdown signal received")
    if 'robot' in globals():
        robot.stop()
        robot.cleanup()
    sys.exit(0)


def main():
    """Main application entry point"""
    logger.info("=" * 60)
    logger.info("Autonomous Mapping Robot - Starting")
    logger.info("=" * 60)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Load configuration
    config = load_config()
    logger.info("Configuration loaded")
    
    # Create robot system
    global robot
    robot = RobotSystem(config)
    
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
        robot.slam_system,
        robot.stereo_camera,
        robot.autonomous_mapper
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