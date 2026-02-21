"""
Sensor Fusion Module
Combines Visual Odometry from SLAM with IMU data for improved tracking accuracy
"""
import numpy as np
import logging
import time
import threading
from typing import Optional, Tuple
from collections import deque

logger = logging.getLogger(__name__)


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for fusing visual odometry and IMU data
    State: [x, y, theta, vx, vy, omega]
    """
    
    def __init__(self):
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        
        # State covariance matrix
        self.P = np.eye(6) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise covariance (will be set based on sensor)
        self.R_visual = np.diag([0.1, 0.1, 0.2])  # Visual odometry: x, y, theta
        self.R_imu = np.diag([0.01])  # IMU: omega (angular velocity)
        
        self.last_update_time = None
    
    def predict(self, dt: float, gyro_z: float):
        """
        Prediction step using IMU gyroscope
        
        Args:
            dt: Time step
            gyro_z: Gyroscope Z-axis (yaw rate) in rad/s
        """
        # State transition (simple motion model)
        x, y, theta, vx, vy, omega = self.state
        
        # Predict new state
        new_x = x + (vx * np.cos(theta) - vy * np.sin(theta)) * dt
        new_y = y + (vx * np.sin(theta) + vy * np.cos(theta)) * dt
        new_theta = theta + omega * dt
        new_vx = vx
        new_vy = vy
        new_omega = gyro_z  # Update angular velocity from IMU
        
        self.state = np.array([new_x, new_y, new_theta, new_vx, new_vy, new_omega])
        
        # Normalize theta
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Jacobian of state transition
        F = np.eye(6)
        F[0, 2] = -(vx * np.sin(theta) + vy * np.cos(theta)) * dt
        F[0, 3] = np.cos(theta) * dt
        F[0, 4] = -np.sin(theta) * dt
        F[1, 2] = (vx * np.cos(theta) - vy * np.sin(theta)) * dt
        F[1, 3] = np.sin(theta) * dt
        F[1, 4] = np.cos(theta) * dt
        F[2, 5] = dt
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_visual(self, measurement: Tuple[float, float, float]):
        """
        Update step using visual odometry measurement
        
        Args:
            measurement: (x, y, theta) from visual SLAM
        """
        # Measurement model: H = [1, 0, 0, 0, 0, 0]
        #                         [0, 1, 0, 0, 0, 0]
        #                         [0, 0, 1, 0, 0, 0]
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        
        # Measurement
        z = np.array(measurement)
        
        # Predicted measurement
        h = self.state[:3]
        
        # Innovation
        y = z - h
        
        # Normalize angle difference
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_visual
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Normalize theta
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose estimate (x, y, theta)"""
        return self.state[0], self.state[1], self.state[2]
    
    def get_velocity(self) -> Tuple[float, float, float]:
        """Get current velocity estimate (vx, vy, omega)"""
        return self.state[3], self.state[4], self.state[5]
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset filter state"""
        self.state = np.array([x, y, theta, 0.0, 0.0, 0.0])
        self.P = np.eye(6) * 0.1


class IMUVisualFusion:
    """
    Sensor fusion combining Visual SLAM with IMU for improved orientation tracking
    """
    
    def __init__(self, 
                 use_ekf: bool = True,
                 imu_weight: float = 0.3):
        """
        Initialize sensor fusion
        
        Args:
            use_ekf: Use Extended Kalman Filter (True) or simple complementary filter (False)
            imu_weight: Weight for IMU in complementary filter (0-1)
        """
        self.use_ekf = use_ekf
        self.imu_weight = imu_weight
        
        # Kalman filter (if enabled)
        self.ekf = ExtendedKalmanFilter() if use_ekf else None
        
        # Simple fusion (if not using EKF)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Tracking
        self.last_update_time = None
        self.last_visual_time = None
        self.last_imu_time = None
        
        # Statistics
        self.visual_updates = 0
        self.imu_updates = 0
        
        # Adaptive fusion
        self.current_visual_quality = 1.0  # Track visual tracking quality
        
        # Thread safety
        self.lock = threading.Lock()
        
        logger.info(f"Sensor fusion initialized (EKF: {use_ekf}, IMU weight: {imu_weight})")
    
    def update_from_visual(self, x: float, y: float, theta: float, quality: float = 1.0):
        """
        Update pose from visual odometry (SLAM)
        
        Args:
            x, y: Position in meters
            theta: Orientation in radians
            quality: Visual tracking quality (0.0-1.0, higher is better)
        """
        with self.lock:
            current_time = time.time()
            self.current_visual_quality = quality
            
            if self.use_ekf and self.ekf:
                # Update EKF with visual measurement
                # Adjust measurement noise based on quality
                original_R = self.ekf.R_visual.copy()
                self.ekf.R_visual = original_R * (2.0 - quality)  # Lower quality = higher noise
                self.ekf.update_visual((x, y, theta))
                self.ekf.R_visual = original_R  # Restore
                self.x, self.y, self.theta = self.ekf.get_pose()
            else:
                # Simple update (weighted average if IMU data is recent)
                # Adjust IMU weight based on visual quality
                adaptive_imu_weight = self.imu_weight + (1.0 - quality) * 0.3
                adaptive_imu_weight = np.clip(adaptive_imu_weight, 0.0, 0.9)
                
                if self.last_imu_time and (current_time - self.last_imu_time < 0.1):
                    # Recent IMU data available, blend orientations
                    visual_weight = 1.0 - adaptive_imu_weight
                    
                    # Use circular mean for angles
                    self.theta = np.arctan2(
                        np.sin(theta) * visual_weight + np.sin(self.theta) * adaptive_imu_weight,
                        np.cos(theta) * visual_weight + np.cos(self.theta) * adaptive_imu_weight
                    )
                else:
                    # No recent IMU data, use visual directly
                    self.theta = theta
                
                # Position from visual odometry
                self.x = x
                self.y = y
            
            self.visual_updates += 1
            self.last_visual_time = current_time
            self.last_update_time = current_time
    
    def update_from_imu(self, gyro_z: float):
        """
        Update orientation from IMU gyroscope
        
        Args:
            gyro_z: Angular velocity around Z-axis (yaw rate) in rad/s
        """
        with self.lock:
            current_time = time.time()
            
            if self.last_update_time is None:
                self.last_update_time = current_time
                self.last_imu_time = current_time
                return
            
            dt = current_time - self.last_update_time
            
            if self.use_ekf and self.ekf:
                # Predict step with IMU
                self.ekf.predict(dt, gyro_z)
                self.x, self.y, self.theta = self.ekf.get_pose()
            else:
                # Simple integration
                self.theta += gyro_z * dt
                self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
            
            self.imu_updates += 1
            self.last_imu_time = current_time
            self.last_update_time = current_time
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get current fused pose estimate
        
        Returns:
            (x, y, theta) in meters and radians
        """
        with self.lock:
            return self.x, self.y, self.theta
    
    def get_2d_pose(self) -> Tuple[float, float, float]:
        """Alias for get_pose() for compatibility with SLAM interface"""
        return self.get_pose()
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset fusion state"""
        with self.lock:
            if self.ekf:
                self.ekf.reset(x, y, theta)
            self.x = x
            self.y = y
            self.theta = theta
            self.last_update_time = None
            self.visual_quality = self.current_visual_quality
            self.last_visual_time = None
            self.last_imu_time = None
            logger.info(f"Sensor fusion reset to ({x}, {y}, {theta})")
    
    def get_statistics(self) -> dict:
        """Get fusion statistics"""
        with self.lock:
            return {
                'visual_updates': self.visual_updates,
                'imu_updates': self.imu_updates,
                'last_visual_age': time.time() - self.last_visual_time if self.last_visual_time else None,
                'last_imu_age': time.time() - self.last_imu_time if self.last_imu_time else None,
                'pose': (self.x, self.y, self.theta)
            }


class FusedSLAMWrapper:
    """
    Wrapper that combines SLAM system with IMU for improved tracking
    Acts as a drop-in replacement for regular SLAM system
    """
    
    def __init__(self, 
                 slam_system,
                 imu_sensor,
                 use_ekf: bool = True,
                 imu_weight: float = 0.3):
        """
        Initialize fused SLAM wrapper
        
        Args:
            slam_system: Underlying SLAM system (ORBSLAM3Wrapper or SimpleVisualSLAM)
            imu_sensor: MPU6050 IMU sensor
            use_ekf: Use Extended Kalman Filter for fusion
            imu_weight: Weight for IMU in complementary filter
        """
        self.slam = slam_system
        self.imu = imu_sensor
        self.fusion = IMUVisualFusion(use_ekf=use_ekf, imu_weight=imu_weight)
        
        # Background fusion thread
        self.running = False
        self.fusion_thread = None
        
        logger.info("Fused SLAM wrapper initialized")
    
    def start(self):
        """Start fusion thread"""
        if not self.running:
            self.running = True
            self.fusion_thread = threading.Thread(target=self._fusion_loop, daemon=True)
            self.fusion_thread.start()
            logger.info("Fused SLAM fusion loop started")
    
    def stop(self):
        """Stop fusion thread"""
        self.running = False
        if self.fusion_thread:
            self.fusion_thread.join(timeout=2.0)
        logger.info("Fused SLAM fusion loop stopped")
    
    def _fusion_loop(self):
        """Continuous fusion of SLAM and IMU data"""
        while self.running:
            try:
                # Get visual odometry from SLAM
                slam_pose = self.slam.get_2d_pose()
                if slam_pose:
                    x, y, theta = slam_pose
                    
                    # Get tracking quality if available
                    quality = 1.0
                    if hasattr(self.slam, 'tracking_quality'):
                        quality = self.slam.tracking_quality
                    elif hasattr(self.slam, 'get_stats'):
                        stats = self.slam.get_stats()
                        quality = stats.get('tracking_quality', 1.0)
                    
                    self.fusion.update_from_visual(x, y, theta, quality=quality)
                
                # Get IMU gyroscope data
                gyro = self.imu.get_gyro()
                gyro_z_rad = np.radians(gyro[2])  # Convert deg/s to rad/s
                self.fusion.update_from_imu(gyro_z_rad)
                
                # Run at ~100Hz
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Fusion loop error: {e}")
                time.sleep(0.1)
    
    def get_2d_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current fused 2D pose
        
        Returns:
            (x, y, theta) or None if not available
        """
        return self.fusion.get_pose()
    
    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Alias for get_2d_pose()"""
        return self.get_2d_pose()
    
    def reset(self):
        """Reset both SLAM and fusion"""
        if hasattr(self.slam, 'reset'):
            self.slam.reset()
        self.fusion.reset()
        logger.info("Fused SLAM system reset")
    
    def get_statistics(self) -> dict:
        """Get fusion statistics"""
        return self.fusion.get_statistics()
    
    # Forward other methods to underlying SLAM
    def __getattr__(self, name):
        """Forward unknown attributes to underlying SLAM system"""
        return getattr(self.slam, name)
