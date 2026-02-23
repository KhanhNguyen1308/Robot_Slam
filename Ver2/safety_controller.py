"""
Safety Velocity Controller
Wraps motor controller with real-time obstacle avoidance for manual control
"""
import logging
from typing import Tuple, Optional, Dict
from obstacle_detector import ObstacleDetector

logger = logging.getLogger(__name__)


class SafetyVelocityController:
    """
    Safety wrapper for velocity commands with real-time obstacle avoidance
    Useful for teleoperation and manual control
    """
    
    def __init__(self, 
                 motor_controller,
                 obstacle_detector: Optional[ObstacleDetector] = None,
                 enable_by_default: bool = True):
        """
        Initialize safety controller
        
        Args:
            motor_controller: Motor controller instance
            obstacle_detector: Obstacle detector instance
            enable_by_default: Whether to enable safety by default
        """
        self.motor = motor_controller
        self.obstacle_detector = obstacle_detector
        self.enabled = enable_by_default
        
        # Latest obstacle detection result
        self.latest_detection = None
        
        # Statistics
        self.commands_sent = 0
        self.commands_modified = 0
        self.commands_blocked = 0
        
        logger.info(f"SafetyVelocityController initialized (enabled={enable_by_default})")
    
    def send_velocity(self, 
                     linear: float, 
                     angular: float,
                     force: bool = False) -> bool:
        """
        Send velocity command with safety checking
        
        Args:
            linear: Desired linear velocity (m/s)
            angular: Desired angular velocity (rad/s)
            force: If True, bypass safety checks (use with caution!)
            
        Returns:
            True if command was sent successfully
        """
        self.commands_sent += 1
        
        # If safety disabled or forced, send directly
        if not self.enabled or force:
            return self.motor.send_velocity(linear, angular)
        
        # Apply safety limitations
        safe_linear, safe_angular = self._apply_safety(linear, angular)
        
        # Track if command was modified
        if abs(safe_linear - linear) > 0.01 or abs(safe_angular - angular) > 0.01:
            self.commands_modified += 1
            logger.debug(f"Safety: modified velocity from ({linear:.2f}, {angular:.2f}) "
                        f"to ({safe_linear:.2f}, {safe_angular:.2f})")
        
        # Track if command was blocked
        if safe_linear == 0.0 and safe_angular == 0.0 and (linear != 0.0 or angular != 0.0):
            self.commands_blocked += 1
            logger.warning(f"Safety: blocked velocity command ({linear:.2f}, {angular:.2f})")
        
        # Send safe velocity
        return self.motor.send_velocity(safe_linear, safe_angular)
    
    def _apply_safety(self, linear: float, angular: float) -> Tuple[float, float]:
        """
        Apply safety constraints based on obstacle detection
        
        Args:
            linear: Desired linear velocity
            angular: Desired angular velocity
            
        Returns:
            (safe_linear, safe_angular) tuple
        """
        # If no obstacle detector or no recent detection, return as-is
        if not self.obstacle_detector or not self.latest_detection:
            return linear, angular
        
        # Use obstacle detector's safety velocity calculation
        safe_linear, safe_angular = self.obstacle_detector.get_safety_velocity(
            linear, angular, self.latest_detection
        )
        
        return safe_linear, safe_angular
    
    def update_obstacle_detection(self, detection_result: Dict):
        """
        Update latest obstacle detection result
        
        Args:
            detection_result: Result from ObstacleDetector.detect_from_disparity()
        """
        self.latest_detection = detection_result
    
    def enable(self):
        """Enable safety checking"""
        self.enabled = True
        logger.info("Safety velocity controller ENABLED")
    
    def disable(self):
        """Disable safety checking (use with caution!)"""
        self.enabled = False
        logger.warning("Safety velocity controller DISABLED - use caution!")
    
    def is_enabled(self) -> bool:
        """Check if safety is enabled"""
        return self.enabled
    
    def get_obstacle_status(self) -> Optional[Dict]:
        """
        Get current obstacle status
        
        Returns:
            Dict with obstacle information or None
        """
        if not self.latest_detection:
            return None
        
        action = self.latest_detection.get('action', {})
        
        return {
            'has_obstacles': action.get('warning', False),
            'severity': action.get('severity', 'none'),
            'message': action.get('message', 'No obstacles'),
            'closest_distance': self.latest_detection.get('closest_obstacle_distance'),
            'stop_required': action.get('stop_required', False),
            'clear_directions': self.latest_detection.get('clear_directions', [])
        }
    
    def get_stats(self) -> Dict:
        """Get safety controller statistics"""
        modification_rate = 0.0
        blocking_rate = 0.0
        
        if self.commands_sent > 0:
            modification_rate = (self.commands_modified / self.commands_sent) * 100
            blocking_rate = (self.commands_blocked / self.commands_sent) * 100
        
        return {
            'enabled': self.enabled,
            'commands_sent': self.commands_sent,
            'commands_modified': self.commands_modified,
            'commands_blocked': self.commands_blocked,
            'modification_rate': modification_rate,
            'blocking_rate': blocking_rate,
            'has_detector': self.obstacle_detector is not None,
            'has_detection': self.latest_detection is not None
        }
    
    def reset_stats(self):
        """Reset statistics"""
        self.commands_sent = 0
        self.commands_modified = 0
        self.commands_blocked = 0
        logger.info("Safety controller statistics reset")
    
    # Proxy methods to motor controller
    def stop(self):
        """Emergency stop (bypasses safety checks)"""
        return self.motor.stop()
    
    def get_status(self):
        """Get motor controller status"""
        return self.motor.get_status()
    
    def __getattr__(self, name):
        """Forward other calls to motor controller"""
        return getattr(self.motor, name)
