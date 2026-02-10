"""
Obstacle Detection Module
Uses stereo depth estimation and simple detection algorithms
"""
import cv2
import numpy as np
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class Obstacle:
    """Obstacle information"""
    distance: float  # meters
    angle: float  # degrees from center (-90 to 90)
    width: int  # pixels
    height: int  # pixels
    center_x: int
    center_y: int
    severity: str  # 'low', 'medium', 'high'


class ObstacleDetector:
    """
    Obstacle detection using stereo depth map
    """
    
    def __init__(self, 
                 camera_baseline: float,
                 camera_focal_length: float,
                 min_distance: float = 0.2,
                 max_distance: float = 3.0,
                 danger_zone_distance: float = 0.5):
        """
        Args:
            camera_baseline: Distance between cameras in meters
            camera_focal_length: Camera focal length in pixels
            min_distance: Minimum detection distance in meters
            max_distance: Maximum detection distance in meters
            danger_zone_distance: Distance threshold for danger zone
        """
        self.baseline = camera_baseline
        self.focal_length = camera_focal_length
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.danger_zone = danger_zone_distance
        
        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
        # SGBM for better quality (slower)
        self.stereo_sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*5,
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        
        # Detection parameters
        self.roi_bottom_margin = 0.3  # Ignore bottom 30% (ground)
        self.roi_top_margin = 0.1     # Ignore top 10% (sky)
        
        # History for temporal filtering
        self.obstacle_history = []
        self.history_size = 3
        
    def compute_disparity(self, 
                         img_left: np.ndarray, 
                         img_right: np.ndarray,
                         use_sgbm: bool = False) -> np.ndarray:
        """
        Compute disparity map from stereo pair
        
        Args:
            img_left: Left rectified image
            img_right: Right rectified image
            use_sgbm: Use SGBM instead of BM (better quality, slower)
        
        Returns:
            Disparity map (float32)
        """
        # Convert to grayscale
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        # Compute disparity
        if use_sgbm:
            disparity = self.stereo_sgbm.compute(gray_left, gray_right)
        else:
            disparity = self.stereo.compute(gray_left, gray_right)
        
        # Convert to float and scale
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """
        Convert disparity to depth in meters
        
        Formula: depth = (baseline * focal_length) / disparity
        """
        # Avoid division by zero
        disparity[disparity <= 0] = 0.1
        
        depth = (self.baseline * self.focal_length) / disparity
        
        # Clip to valid range
        depth = np.clip(depth, self.min_distance, self.max_distance)
        
        return depth
    
    def detect_obstacles(self, 
                        depth_map: np.ndarray,
                        image: np.ndarray = None) -> List[Obstacle]:
        """
        Detect obstacles from depth map
        
        Args:
            depth_map: Depth map in meters
            image: Original image for visualization (optional)
        
        Returns:
            List of detected obstacles
        """
        height, width = depth_map.shape
        
        # Define ROI (ignore ground and sky)
        roi_top = int(height * self.roi_top_margin)
        roi_bottom = int(height * (1 - self.roi_bottom_margin))
        
        # Extract ROI
        roi_depth = depth_map[roi_top:roi_bottom, :]
        
        # Threshold for close obstacles
        obstacle_mask = (roi_depth < self.danger_zone * 2.0).astype(np.uint8) * 255
        
        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        
        for contour in contours:
            # Filter small contours
            area = cv2.contourArea(contour)
            if area < 500:  # Minimum area threshold
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Adjust y coordinate for full image
            y += roi_top
            
            # Get center point
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Get distance at center
            if 0 <= center_y < height and 0 <= center_x < width:
                distance = depth_map[center_y, center_x]
                
                # Calculate angle from center
                angle = np.degrees(np.arctan2(center_x - width/2, self.focal_length))
                
                # Determine severity
                if distance < self.danger_zone:
                    severity = 'high'
                elif distance < self.danger_zone * 1.5:
                    severity = 'medium'
                else:
                    severity = 'low'
                
                obstacle = Obstacle(
                    distance=float(distance),
                    angle=float(angle),
                    width=w,
                    height=h,
                    center_x=center_x,
                    center_y=center_y,
                    severity=severity
                )
                
                obstacles.append(obstacle)
        
        # Sort by distance (closest first)
        obstacles.sort(key=lambda o: o.distance)
        
        return obstacles
    
    def get_danger_zones(self, obstacles: List[Obstacle]) -> dict:
        """
        Analyze obstacles and return danger zones
        
        Returns:
            dict with 'left', 'center', 'right' danger levels
        """
        zones = {
            'left': 'clear',
            'center': 'clear',
            'right': 'clear'
        }
        
        for obs in obstacles:
            # Determine zone
            if obs.angle < -20:
                zone = 'left'
            elif obs.angle > 20:
                zone = 'right'
            else:
                zone = 'center'
            
            # Update danger level
            if obs.severity == 'high':
                zones[zone] = 'danger'
            elif obs.severity == 'medium' and zones[zone] != 'danger':
                zones[zone] = 'warning'
        
        return zones
    
    def visualize_obstacles(self, 
                           image: np.ndarray,
                           obstacles: List[Obstacle],
                           depth_map: np.ndarray = None) -> np.ndarray:
        """
        Draw obstacles on image
        
        Args:
            image: Input image
            obstacles: List of obstacles
            depth_map: Optional depth map for overlay
        
        Returns:
            Image with obstacles drawn
        """
        vis = image.copy()
        
        # Draw depth map overlay if provided
        if depth_map is not None:
            # Normalize depth for visualization
            depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_color = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
            vis = cv2.addWeighted(vis, 0.7, depth_color, 0.3, 0)
        
        # Draw obstacles
        for obs in obstacles:
            # Color based on severity
            if obs.severity == 'high':
                color = (0, 0, 255)  # Red
            elif obs.severity == 'medium':
                color = (0, 165, 255)  # Orange
            else:
                color = (0, 255, 0)  # Green
            
            # Draw bounding box
            x = obs.center_x - obs.width // 2
            y = obs.center_y - obs.height // 2
            cv2.rectangle(vis, (x, y), (x + obs.width, y + obs.height), color, 2)
            
            # Draw distance and angle
            label = f"{obs.distance:.2f}m {obs.angle:.0f}°"
            cv2.putText(vis, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw danger zones
        height, width = vis.shape[:2]
        
        # Left zone
        cv2.rectangle(vis, (0, 0), (width//3, 30), (50, 50, 50), -1)
        cv2.putText(vis, "LEFT", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Center zone
        cv2.rectangle(vis, (width//3, 0), (2*width//3, 30), (50, 50, 50), -1)
        cv2.putText(vis, "CENTER", (width//3 + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Right zone
        cv2.rectangle(vis, (2*width//3, 0), (width, 30), (50, 50, 50), -1)
        cv2.putText(vis, "RIGHT", (2*width//3 + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return vis
    
    def get_safe_direction(self, obstacles: List[Obstacle]) -> Tuple[str, float]:
        """
        Determine safest direction to move
        
        Returns:
            (direction, confidence) where direction is 'forward', 'left', 'right', 'backward'
        """
        zones = self.get_danger_zones(obstacles)
        
        # Simple logic
        if zones['center'] == 'clear':
            return ('forward', 1.0)
        elif zones['left'] == 'clear' and zones['right'] != 'clear':
            return ('left', 0.8)
        elif zones['right'] == 'clear' and zones['left'] != 'clear':
            return ('right', 0.8)
        elif zones['left'] == 'clear':
            return ('left', 0.6)
        elif zones['right'] == 'clear':
            return ('right', 0.6)
        else:
            return ('backward', 0.5)


class SimpleObstacleAvoidance:
    """
    Simple reactive obstacle avoidance behavior
    """
    
    def __init__(self, detector: ObstacleDetector):
        self.detector = detector
        
        # Control parameters
        self.normal_speed = 0.3  # m/s
        self.slow_speed = 0.15
        self.turn_speed = 1.0  # rad/s
        
    def compute_velocity(self, obstacles: List[Obstacle]) -> Tuple[float, float]:
        """
        Compute velocity commands based on obstacles
        
        Returns:
            (linear, angular) velocity
        """
        if not obstacles:
            # No obstacles - go forward
            return (self.normal_speed, 0.0)
        
        # Get danger zones
        zones = self.detector.get_danger_zones(obstacles)
        
        # Find closest obstacle
        closest = obstacles[0]
        
        # Emergency stop if very close
        if closest.distance < 0.3:
            return (0.0, 0.0)
        
        # Slow down if obstacle ahead
        if zones['center'] == 'danger':
            # Turn away from obstacle
            if closest.angle < 0:
                # Obstacle on left, turn right
                return (0.0, -self.turn_speed)
            else:
                # Obstacle on right, turn left
                return (0.0, self.turn_speed)
        
        elif zones['center'] == 'warning':
            # Slow down and adjust
            angular = -np.sign(closest.angle) * self.turn_speed * 0.5
            return (self.slow_speed, angular)
        
        else:
            # Path is clear
            return (self.normal_speed, 0.0)


# Example usage
if __name__ == '__main__':
    import time
    
    logging.basicConfig(level=logging.INFO)
    
    # Simulated parameters
    baseline = 0.06  # 6cm
    focal_length = 700  # pixels
    
    detector = ObstacleDetector(baseline, focal_length)
    
    # Test with dummy data
    print("Obstacle Detection Test")
    print("=" * 50)
    
    # Create dummy depth map
    depth = np.ones((480, 640)) * 2.0  # 2m everywhere
    
    # Add some obstacles
    depth[200:300, 250:350] = 0.4  # Close obstacle in center
    depth[150:250, 100:200] = 1.0  # Medium obstacle on left
    
    # Detect
    obstacles = detector.detect_obstacles(depth)
    
    print(f"Found {len(obstacles)} obstacles:")
    for i, obs in enumerate(obstacles):
        print(f"  {i+1}. Distance: {obs.distance:.2f}m, "
              f"Angle: {obs.angle:.1f}°, "
              f"Severity: {obs.severity}")
    
    # Get danger zones
    zones = detector.get_danger_zones(obstacles)
    print(f"\nDanger zones: {zones}")
    
    # Get safe direction
    direction, confidence = detector.get_safe_direction(obstacles)
    print(f"Safe direction: {direction} (confidence: {confidence:.2f})")