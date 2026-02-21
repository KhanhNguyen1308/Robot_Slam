"""
Obstacle Detection Module for Autonomous Navigation
Detects obstacles from stereo depth maps and provides safe navigation commands
"""
import numpy as np
import cv2
import logging
from typing import Optional, Tuple, List, Dict
from collections import deque

logger = logging.getLogger(__name__)


class ObstacleDetector:
    """
    Real-time obstacle detection from stereo depth maps
    Provides collision avoidance for autonomous navigation
    """
    
    def __init__(self,
                 min_safe_distance: float = 0.3,      # meters
                 critical_distance: float = 0.15,      # meters
                 detection_width: float = 0.5,         # meters (robot width + margin)
                 detection_height: float = 0.4,        # meters (sensor height)
                 max_detection_range: float = 2.0,     # meters
                 obstacle_history_size: int = 5):      # frames to average
        
        self.min_safe_distance = min_safe_distance
        self.critical_distance = critical_distance
        self.detection_width = detection_width
        self.detection_height = detection_height
        self.max_detection_range = max_detection_range
        
        # Detection zones
        self.zones = {
            'front_center': {'angle': (0, 0), 'distance': min_safe_distance},
            'front_left': {'angle': (-45, 0), 'distance': min_safe_distance},
            'front_right': {'angle': (0, 45), 'distance': min_safe_distance},
            'left': {'angle': (-90, -45), 'distance': min_safe_distance * 0.7},
            'right': {'angle': (45, 90), 'distance': min_safe_distance * 0.7}
        }
        
        # Obstacle tracking
        self.obstacle_history = deque(maxlen=obstacle_history_size)
        self.last_clear_direction = 0.0  # Last known clear direction
        self.last_turn_direction = 0.0   # Hysteresis: remember last turn to avoid flip-flopping
        
        # Statistics
        self.obstacles_detected = 0
        self.collision_warnings = 0
        self.emergency_stops = 0
        
        logger.info(f"ObstacleDetector initialized: safe_dist={min_safe_distance}m, "
                   f"critical={critical_distance}m, width={detection_width}m")
    
    def detect_from_disparity(self, 
                             disparity: np.ndarray,
                             camera_matrix: np.ndarray,
                             baseline: float,
                             robot_velocity: Tuple[float, float] = (0.0, 0.0)) -> Dict:
        """
        Detect obstacles from disparity map
        
        Args:
            disparity: Disparity map from stereo camera
            camera_matrix: Camera intrinsic matrix (K)
            baseline: Stereo baseline in meters
            robot_velocity: Current (linear, angular) velocity
            
        Returns:
            Detection result dict with obstacle info and recommended action
        """
        if disparity is None or disparity.size == 0:
            return self._create_no_detection_result()
        
        # Convert disparity to depth
        depth_map = self._disparity_to_depth(disparity, camera_matrix, baseline)
        
        # Extract obstacle information from depth map
        obstacles = self._extract_obstacles(depth_map, camera_matrix)
        
        # Analyze zones
        zone_status = self._analyze_zones(obstacles, depth_map)
        
        # Determine action based on obstacles and current velocity
        action = self._determine_action(zone_status, robot_velocity)
        
        # Track obstacle history
        self.obstacle_history.append(zone_status)
        
        # Update statistics
        if any(zone['has_obstacle'] for zone in zone_status.values()):
            self.obstacles_detected += 1
        
        if action['stop_required']:
            self.emergency_stops += 1
        elif action['warning']:
            self.collision_warnings += 1
        
        result = {
            'obstacles': obstacles,
            'zone_status': zone_status,
            'action': action,
            'depth_map': depth_map,
            'closest_obstacle_distance': self._get_closest_distance(zone_status),
            'clear_directions': self._find_clear_directions(zone_status)
        }
        
        return result
    
    def _disparity_to_depth(self, 
                           disparity: np.ndarray,
                           camera_matrix: np.ndarray,
                           baseline: float) -> np.ndarray:
        """
        Convert disparity to depth map
        
        Formula: depth = (focal_length * baseline) / disparity
        """
        # Get focal length from camera matrix
        fx = camera_matrix[0, 0]
        
        # Avoid division by zero
        valid_disparity = disparity > 0
        
        # Create depth map
        depth = np.zeros_like(disparity, dtype=np.float32)
        depth[valid_disparity] = (fx * baseline) / disparity[valid_disparity]
        
        # Clip to valid range
        depth = np.clip(depth, 0, self.max_detection_range)
        
        return depth
    
    def _extract_obstacles(self, 
                          depth_map: np.ndarray,
                          camera_matrix: np.ndarray) -> List[Dict]:
        """
        Extract individual obstacle blobs from depth map
        """
        height, width = depth_map.shape
        
        # Create obstacle mask (objects within detection range)
        obstacle_mask = (depth_map > 0) & (depth_map < self.min_safe_distance)
        
        # Morphological operations to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        obstacle_mask = cv2.morphologyEx(obstacle_mask.astype(np.uint8), 
                                        cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        # Find connected components (obstacles)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            obstacle_mask, connectivity=8)
        
        obstacles = []
        
        # Skip background (label 0)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            
            # Filter small blobs (noise)
            if area < 50:  # Minimum 50 pixels
                continue
            
            # Get obstacle region
            mask = (labels == i)
            obstacle_depths = depth_map[mask]
            
            if len(obstacle_depths) == 0:
                continue
            
            # Calculate obstacle properties
            min_distance = np.min(obstacle_depths)
            mean_distance = np.mean(obstacle_depths)
            
            cx, cy = centroids[i]
            
            # Convert pixel position to angle
            fx = camera_matrix[0, 0]
            cx_cam = camera_matrix[0, 2]
            
            # Angle from camera center
            angle = np.arctan2(cx - cx_cam, fx)
            angle_deg = np.degrees(angle)
            
            obstacle = {
                'distance': float(min_distance),
                'mean_distance': float(mean_distance),
                'angle': float(angle_deg),
                'size': int(area),
                'position': (float(cx), float(cy)),
                'critical': min_distance < self.critical_distance
            }
            
            obstacles.append(obstacle)
        
        # Sort by distance (closest first)
        obstacles.sort(key=lambda x: x['distance'])
        
        return obstacles
    
    def _analyze_zones(self, 
                      obstacles: List[Dict],
                      depth_map: np.ndarray) -> Dict[str, Dict]:
        """
        Analyze each detection zone for obstacles
        """
        zone_status = {}
        
        for zone_name, zone_def in self.zones.items():
            angle_min, angle_max = zone_def['angle']
            safe_dist = zone_def['distance']
            
            # Find obstacles in this zone
            zone_obstacles = [
                obs for obs in obstacles
                if angle_min <= obs['angle'] <= angle_max
            ]
            
            has_obstacle = False
            min_dist = float('inf')
            critical = False
            
            if zone_obstacles:
                min_dist = min(obs['distance'] for obs in zone_obstacles)
                has_obstacle = min_dist < safe_dist
                critical = min_dist < self.critical_distance
            
            zone_status[zone_name] = {
                'has_obstacle': has_obstacle,
                'critical': critical,
                'min_distance': min_dist if min_dist != float('inf') else None,
                'obstacle_count': len(zone_obstacles),
                'safe': not has_obstacle
            }
        
        return zone_status
    
    def _determine_action(self,
                         zone_status: Dict[str, Dict],
                         robot_velocity: Tuple[float, float]) -> Dict:
        """
        Determine recommended action based on zone status
        
        Returns:
            Dict with action recommendations
        """
        linear, angular = robot_velocity
        
        # Check for critical obstacles (emergency stop)
        critical_zones = [name for name, status in zone_status.items() 
                         if status.get('critical', False)]
        
        if critical_zones:
            return {
                'stop_required': True,
                'warning': True,
                'recommended_linear': 0.0,
                'recommended_angular': 0.0,
                'message': f"EMERGENCY STOP: Critical obstacle in {', '.join(critical_zones)}",
                'severity': 'critical'
            }
        
        # Check front zones for obstacles
        front_blocked = (zone_status['front_center']['has_obstacle'] or
                        (zone_status['front_left']['has_obstacle'] and 
                         zone_status['front_right']['has_obstacle']))
        
        if front_blocked and linear > 0:
            # Front is blocked, need to slow down or turn
            
            # Check which side is clearer
            left_clear = not zone_status['front_left']['has_obstacle']
            right_clear = not zone_status['front_right']['has_obstacle']
            
            # HYSTERESIS: Prefer continuing in the last turn direction to avoid oscillation
            # Only change direction if the preferred side is clearly blocked
            if left_clear and not right_clear:
                # Turn left
                self.last_turn_direction = 0.5
                return {
                    'stop_required': False,
                    'warning': True,
                    'recommended_linear': linear * 0.3,  # Slow down
                    'recommended_angular': 0.5,  # Turn left
                    'message': "Obstacle ahead - turning left",
                    'severity': 'warning'
                }
            elif right_clear and not left_clear:
                # Turn right
                self.last_turn_direction = -0.5
                return {
                    'stop_required': False,
                    'warning': True,
                    'recommended_linear': linear * 0.3,
                    'recommended_angular': -0.5,  # Turn right
                    'message': "Obstacle ahead - turning right",
                    'severity': 'warning'
                }
            elif left_clear and right_clear:
                # Both sides clear - use hysteresis to maintain direction
                if abs(self.last_turn_direction) > 0:
                    # Continue in last direction
                    return {
                        'stop_required': False,
                        'warning': True,
                        'recommended_linear': linear * 0.5,
                        'recommended_angular': self.last_turn_direction,
                        'message': "Obstacle ahead - continuing turn",
                        'severity': 'warning'
                    }
                # No previous preference, default to left
                self.last_turn_direction = 0.5
                return {
                    'stop_required': False,
                    'warning': True,
                    'recommended_linear': linear * 0.5,
                    'recommended_angular': 0.5,
                    'message': "Obstacle ahead - turning left (default)",
                    'severity': 'warning'
                }
            else:
                # Both sides blocked
                return {
                    'stop_required': True,
                    'warning': True,
                    'recommended_linear': 0.0,
                    'recommended_angular': 0.0,
                    'message': "Path blocked - stopping",
                    'severity': 'warning'
                }
        
        # Check side zones when turning
        if angular > 0 and zone_status['left']['has_obstacle']:
            # Turning left but left side has obstacle
            return {
                'stop_required': False,
                'warning': True,
                'recommended_linear': linear * 0.5,
                'recommended_angular': angular * 0.5,
                'message': "Obstacle on left - reducing turn rate",
                'severity': 'caution'
            }
        elif angular < 0 and zone_status['right']['has_obstacle']:
            # Turning right but right side has obstacle
            return {
                'stop_required': False,
                'warning': True,
                'recommended_linear': linear * 0.5,
                'recommended_angular': angular * 0.5,
                'message': "Obstacle on right - reducing turn rate",
                'severity': 'caution'
            }
        
        # All clear - reset hysteresis
        self.last_turn_direction = 0.0
        return {
            'stop_required': False,
            'warning': False,
            'recommended_linear': linear,
            'recommended_angular': angular,
            'message': "Path clear",
            'severity': 'none'
        }
    
    def _get_closest_distance(self, zone_status: Dict[str, Dict]) -> Optional[float]:
        """Get distance to closest obstacle across all zones"""
        distances = [status['min_distance'] for status in zone_status.values()
                    if status['min_distance'] is not None]
        return min(distances) if distances else None
    
    def _find_clear_directions(self, zone_status: Dict[str, Dict]) -> List[str]:
        """Find which directions are clear of obstacles"""
        clear_dirs = [name for name, status in zone_status.items() 
                     if status['safe']]
        return clear_dirs
    
    def _create_no_detection_result(self) -> Dict:
        """Create result when no detection is possible"""
        return {
            'obstacles': [],
            'zone_status': {},
            'action': {
                'stop_required': False,
                'warning': False,
                'recommended_linear': 0.0,
                'recommended_angular': 0.0,
                'message': "No depth data available",
                'severity': 'none'
            },
            'depth_map': None,
            'closest_obstacle_distance': None,
            'clear_directions': []
        }
    
    def get_safety_velocity(self,
                           desired_linear: float,
                           desired_angular: float,
                           detection_result: Dict) -> Tuple[float, float]:
        """
        Apply safety limits to desired velocity based on obstacles
        
        Args:
            desired_linear: Desired linear velocity
            desired_angular: Desired angular velocity  
            detection_result: Result from detect_from_disparity()
            
        Returns:
            (safe_linear, safe_angular) velocities
        """
        action = detection_result.get('action', {})
        
        if action.get('stop_required', False):
            return 0.0, 0.0
        
        # Apply recommended velocities if warning
        if action.get('warning', False):
            safe_linear = action.get('recommended_linear', desired_linear * 0.5)
            safe_angular = action.get('recommended_angular', desired_angular * 0.5)
            
            # Never exceed desired velocity
            safe_linear = min(safe_linear, desired_linear)
            safe_angular = min(abs(safe_angular), abs(desired_angular)) * np.sign(desired_angular)
            
            return safe_linear, safe_angular
        
        # No obstacles, return desired velocity
        return desired_linear, desired_angular
    
    def visualize_obstacles(self,
                           image: np.ndarray,
                           detection_result: Dict) -> np.ndarray:
        """
        Draw obstacle detection visualization on image
        
        Args:
            image: Input image (BGR)
            detection_result: Detection result
            
        Returns:
            Image with visualization overlay
        """
        vis_img = image.copy()
        height, width = vis_img.shape[:2]
        
        # Draw zone indicators
        zone_colors = {
            'front_center': (0, 0, 255),   # Red
            'front_left': (0, 140, 255),   # Orange
            'front_right': (0, 140, 255),  # Orange
            'left': (255, 255, 0),         # Cyan
            'right': (255, 255, 0)         # Cyan
        }
        
        zone_status = detection_result.get('zone_status', {})
        
        # Draw status text
        y_offset = 30
        for zone_name, status in zone_status.items():
            color = (0, 255, 0) if status['safe'] else (0, 0, 255)
            if status.get('critical', False):
                color = (255, 0, 255)  # Magenta for critical
            
            dist_text = f"{status['min_distance']:.2f}m" if status['min_distance'] else "Clear"
            text = f"{zone_name}: {dist_text}"
            cv2.putText(vis_img, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_offset += 20
        
        # Draw action message
        action = detection_result.get('action', {})
        message = action.get('message', '')
        severity = action.get('severity', 'none')
        
        msg_color = {
            'critical': (255, 0, 255),
            'warning': (0, 0, 255),
            'caution': (0, 140, 255),
            'none': (0, 255, 0)
        }.get(severity, (255, 255, 255))
        
        cv2.putText(vis_img, message, (10, height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, msg_color, 2)
        
        # Draw obstacles
        obstacles = detection_result.get('obstacles', [])
        for obs in obstacles[:5]:  # Show top 5 closest
            dist = obs['distance']
            angle = obs['angle']
            critical = obs.get('critical', False)
            
            # Draw obstacle marker
            x, y = obs['position']
            color = (255, 0, 255) if critical else (0, 0, 255)
            cv2.circle(vis_img, (int(x), int(y)), 10, color, 2)
            cv2.putText(vis_img, f"{dist:.2f}m", (int(x), int(y) - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return vis_img
    
    def get_stats(self) -> Dict:
        """Get obstacle detection statistics"""
        return {
            'obstacles_detected': self.obstacles_detected,
            'collision_warnings': self.collision_warnings,
            'emergency_stops': self.emergency_stops,
            'history_size': len(self.obstacle_history),
            'min_safe_distance': self.min_safe_distance,
            'critical_distance': self.critical_distance
        }
    
    def reset_stats(self):
        """Reset statistics counters"""
        self.obstacles_detected = 0
        self.collision_warnings = 0
        self.emergency_stops = 0
        self.obstacle_history.clear()
        logger.info("Obstacle detector statistics reset")
