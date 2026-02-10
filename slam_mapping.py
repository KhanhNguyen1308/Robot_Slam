"""
SLAM Mapping & Exploration Module
Autonomous room/library mapping with frontier-based exploration
"""
import numpy as np
import cv2
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import deque
import time

logger = logging.getLogger(__name__)


@dataclass
class MapCell:
    """Single cell in occupancy grid"""
    occupied: float  # 0.0 = free, 1.0 = occupied, 0.5 = unknown
    visited: bool = False
    distance_to_obstacle: float = float('inf')


class OccupancyGrid:
    """
    2D Occupancy Grid Map
    For indoor environment mapping
    """
    
    def __init__(self, 
                 width_meters: float = 10.0,
                 height_meters: float = 10.0,
                 resolution: float = 0.05):
        """
        Args:
            width_meters: Map width in meters
            height_meters: Map height in meters
            resolution: Cell size in meters (5cm default)
        """
        self.width_m = width_meters
        self.height_m = height_meters
        self.resolution = resolution
        
        # Grid dimensions
        self.width = int(width_meters / resolution)
        self.height = int(height_meters / resolution)
        
        # Occupancy grid (0.5 = unknown, 0.0 = free, 1.0 = occupied)
        self.grid = np.ones((self.height, self.width), dtype=np.float32) * 0.5
        
        # Visited map
        self.visited = np.zeros((self.height, self.width), dtype=np.bool_)
        
        # Robot's starting position (center of map)
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2
        
        # Statistics
        self.total_cells = self.width * self.height
        self.explored_cells = 0
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin_x
        grid_y = int(-y / self.resolution) + self.origin_y  # Y is inverted
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (meters)"""
        x = (grid_x - self.origin_x) * self.resolution
        y = -(grid_y - self.origin_y) * self.resolution
        return (x, y)
    
    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def update_from_depth(self, 
                         robot_pose: np.ndarray,
                         depth_map: np.ndarray,
                         camera_params: dict):
        """
        Update occupancy grid from depth map
        
        Args:
            robot_pose: 4x4 transformation matrix
            depth_map: Depth image in meters
            camera_params: Camera parameters (focal_length, width, height)
        """
        # Extract robot position
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        robot_yaw = np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
        
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)
        
        if not self.is_valid(robot_grid_x, robot_grid_y):
            logger.warning("Robot position out of map bounds")
            return
        
        # Mark robot position as free
        self.grid[robot_grid_y, robot_grid_x] = 0.0
        self.visited[robot_grid_y, robot_grid_x] = True
        
        # Process depth map
        h, w = depth_map.shape
        fx = camera_params.get('focal_length', 700)
        
        # Sample depth map (every 10 pixels for efficiency)
        step = 10
        
        for v in range(0, h, step):
            for u in range(0, w, step):
                depth = depth_map[v, u]
                
                # Skip invalid depths
                if depth < 0.1 or depth > 5.0 or np.isnan(depth) or np.isinf(depth):
                    continue
                
                # Convert pixel to 3D point (simplified pinhole model)
                # Assume camera center is at image center
                x_cam = (u - w/2) * depth / fx
                y_cam = depth  # Forward is Y in camera frame
                
                # Transform to world coordinates
                cos_yaw = np.cos(robot_yaw)
                sin_yaw = np.sin(robot_yaw)
                
                x_world = robot_x + x_cam * cos_yaw - y_cam * sin_yaw
                y_world = robot_y + x_cam * sin_yaw + y_cam * cos_yaw
                
                # Convert to grid
                obs_grid_x, obs_grid_y = self.world_to_grid(x_world, y_world)
                
                if not self.is_valid(obs_grid_x, obs_grid_y):
                    continue
                
                # Ray tracing: mark cells along ray as free
                self._bresenham_line(robot_grid_x, robot_grid_y, 
                                    obs_grid_x, obs_grid_y, 
                                    mark_free=True)
                
                # Mark endpoint as occupied
                if depth < 4.0:  # Only mark obstacles, not far points
                    self.grid[obs_grid_y, obs_grid_x] = min(1.0, self.grid[obs_grid_y, obs_grid_x] + 0.1)
        
        # Update exploration statistics
        self.explored_cells = np.sum(self.grid != 0.5)
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int, mark_free: bool = True):
        """Bresenham's line algorithm for ray tracing"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if self.is_valid(x, y):
                if mark_free and (x, y) != (x1, y1):
                    # Mark as free (reduce occupancy)
                    self.grid[y, x] = max(0.0, self.grid[y, x] - 0.05)
                    self.visited[y, x] = True
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def get_exploration_progress(self) -> float:
        """Get exploration progress (0.0 to 1.0)"""
        return self.explored_cells / self.total_cells
    
    def get_occupancy_image(self, 
                           robot_pose: Optional[np.ndarray] = None,
                           path: Optional[List[Tuple[float, float]]] = None) -> np.ndarray:
        """
        Get visualization of occupancy grid
        
        Args:
            robot_pose: Current robot pose (for visualization)
            path: Planned path (for visualization)
        
        Returns:
            RGB image of map
        """
        # Create RGB image
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Color map: white=free, black=occupied, gray=unknown
        for y in range(self.height):
            for x in range(self.width):
                val = self.grid[y, x]
                if val < 0.3:  # Free
                    img[y, x] = [255, 255, 255]
                elif val > 0.7:  # Occupied
                    img[y, x] = [0, 0, 0]
                else:  # Unknown
                    img[y, x] = [128, 128, 128]
        
        # Draw robot position
        if robot_pose is not None:
            robot_x = robot_pose[0, 3]
            robot_y = robot_pose[1, 3]
            robot_yaw = np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
            
            robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)
            
            if self.is_valid(robot_grid_x, robot_grid_y):
                # Draw robot as circle
                cv2.circle(img, (robot_grid_x, robot_grid_y), 5, (0, 255, 0), -1)
                
                # Draw direction arrow
                arrow_len = 10
                end_x = int(robot_grid_x + arrow_len * np.cos(robot_yaw))
                end_y = int(robot_grid_y - arrow_len * np.sin(robot_yaw))
                cv2.arrowedLine(img, (robot_grid_x, robot_grid_y), 
                              (end_x, end_y), (0, 255, 0), 2)
        
        # Draw path
        if path is not None and len(path) > 1:
            for i in range(len(path) - 1):
                x1, y1 = self.world_to_grid(path[i][0], path[i][1])
                x2, y2 = self.world_to_grid(path[i+1][0], path[i+1][1])
                
                if self.is_valid(x1, y1) and self.is_valid(x2, y2):
                    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        
        # Add grid lines for better visualization
        for i in range(0, self.width, int(1.0 / self.resolution)):  # Every 1 meter
            cv2.line(img, (i, 0), (i, self.height), (200, 200, 200), 1)
        for i in range(0, self.height, int(1.0 / self.resolution)):
            cv2.line(img, (0, i), (self.width, i), (200, 200, 200), 1)
        
        return img
    
    def find_frontiers(self) -> List[Tuple[int, int]]:
        """
        Find frontier cells (boundary between explored and unexplored)
        Used for exploration
        
        Returns:
            List of (grid_x, grid_y) frontier cells
        """
        frontiers = []
        
        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                # Cell must be free and visited
                if self.grid[y, x] > 0.3 or not self.visited[y, x]:
                    continue
                
                # Check if any neighbor is unknown
                has_unknown_neighbor = False
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        
                        ny, nx = y + dy, x + dx
                        if self.is_valid(nx, ny):
                            if self.grid[ny, nx] == 0.5:  # Unknown
                                has_unknown_neighbor = True
                                break
                    if has_unknown_neighbor:
                        break
                
                if has_unknown_neighbor:
                    frontiers.append((x, y))
        
        return frontiers
    
    def cluster_frontiers(self, 
                         frontiers: List[Tuple[int, int]],
                         min_cluster_size: int = 10) -> List[List[Tuple[int, int]]]:
        """
        Cluster nearby frontier cells
        
        Args:
            frontiers: List of frontier cells
            min_cluster_size: Minimum cells per cluster
        
        Returns:
            List of frontier clusters
        """
        if len(frontiers) == 0:
            return []
        
        # Simple clustering using connected components
        visited = set()
        clusters = []
        
        for frontier in frontiers:
            if frontier in visited:
                continue
            
            # BFS to find connected frontiers
            cluster = []
            queue = deque([frontier])
            visited.add(frontier)
            
            while queue:
                x, y = queue.popleft()
                cluster.append((x, y))
                
                # Check neighbors
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        
                        neighbor = (x + dx, y + dy)
                        if neighbor in frontiers and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            
            # Only keep large enough clusters
            if len(cluster) >= min_cluster_size:
                clusters.append(cluster)
        
        return clusters
    
    def save_map(self, filename: str):
        """Save map to file"""
        # Save as image
        img = self.get_occupancy_image()
        cv2.imwrite(filename + '.png', img)
        
        # Save grid data
        np.savez(filename + '.npz', 
                grid=self.grid,
                visited=self.visited,
                width=self.width,
                height=self.height,
                resolution=self.resolution,
                origin_x=self.origin_x,
                origin_y=self.origin_y)
        
        logger.info(f"Map saved to {filename}")
    
    def load_map(self, filename: str):
        """Load map from file"""
        data = np.load(filename + '.npz')
        
        self.grid = data['grid']
        self.visited = data['visited']
        self.width = int(data['width'])
        self.height = int(data['height'])
        self.resolution = float(data['resolution'])
        self.origin_x = int(data['origin_x'])
        self.origin_y = int(data['origin_y'])
        
        self.explored_cells = np.sum(self.grid != 0.5)
        
        logger.info(f"Map loaded from {filename}")


class FrontierExplorer:
    """
    Frontier-based exploration strategy
    Guides robot to explore unknown areas
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid):
        self.map = occupancy_grid
        self.exploration_complete = False
        self.current_goal = None
        
    def get_next_goal(self, robot_pose: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        Get next exploration goal using frontier-based exploration
        
        Args:
            robot_pose: Current robot pose
        
        Returns:
            (x, y) goal position in world coordinates, or None if exploration complete
        """
        # Find frontiers
        frontiers = self.map.find_frontiers()
        
        if len(frontiers) == 0:
            logger.info("No frontiers found - exploration complete!")
            self.exploration_complete = True
            return None
        
        # Cluster frontiers
        clusters = self.map.cluster_frontiers(frontiers, min_cluster_size=15)
        
        if len(clusters) == 0:
            logger.info("No significant frontier clusters - exploration complete!")
            self.exploration_complete = True
            return None
        
        # Get robot position
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        robot_grid_x, robot_grid_y = self.map.world_to_grid(robot_x, robot_y)
        
        # Find closest frontier cluster
        best_cluster = None
        best_distance = float('inf')
        
        for cluster in clusters:
            # Get cluster center
            center_x = int(np.mean([c[0] for c in cluster]))
            center_y = int(np.mean([c[1] for c in cluster]))
            
            # Distance to cluster
            dist = np.sqrt((center_x - robot_grid_x)**2 + (center_y - robot_grid_y)**2)
            
            if dist < best_distance:
                best_distance = dist
                best_cluster = (center_x, center_y)
        
        if best_cluster is None:
            return None
        
        # Convert to world coordinates
        goal_x, goal_y = self.map.grid_to_world(best_cluster[0], best_cluster[1])
        self.current_goal = (goal_x, goal_y)
        
        logger.info(f"Next exploration goal: ({goal_x:.2f}, {goal_y:.2f}), "
                   f"distance: {best_distance * self.map.resolution:.2f}m")
        
        return self.current_goal
    
    def is_goal_reached(self, robot_pose: np.ndarray, threshold: float = 0.3) -> bool:
        """Check if robot reached current goal"""
        if self.current_goal is None:
            return True
        
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        
        distance = np.sqrt((robot_x - self.current_goal[0])**2 + 
                          (robot_y - self.current_goal[1])**2)
        
        return distance < threshold


# Example usage and testing
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    print("Testing Occupancy Grid and Frontier Exploration")
    print("=" * 60)
    
    # Create map
    map_grid = OccupancyGrid(width_meters=10.0, height_meters=10.0, resolution=0.05)
    
    print(f"Map size: {map_grid.width}x{map_grid.height} cells")
    print(f"Resolution: {map_grid.resolution}m/cell")
    print(f"Total cells: {map_grid.total_cells}")
    
    # Simulate robot at origin
    robot_pose = np.eye(4)
    
    # Simulate depth measurements
    depth_map = np.ones((480, 640)) * 2.0  # 2m everywhere
    depth_map[200:300, 250:350] = 0.5  # Wall ahead
    
    camera_params = {'focal_length': 700, 'width': 640, 'height': 480}
    
    # Update map
    print("\nUpdating map from depth...")
    map_grid.update_from_depth(robot_pose, depth_map, camera_params)
    
    print(f"Exploration: {map_grid.get_exploration_progress()*100:.1f}%")
    print(f"Explored cells: {map_grid.explored_cells}")
    
    # Find frontiers
    print("\nFinding frontiers...")
    frontiers = map_grid.find_frontiers()
    print(f"Found {len(frontiers)} frontier cells")
    
    # Cluster frontiers
    clusters = map_grid.cluster_frontiers(frontiers)
    print(f"Found {len(clusters)} frontier clusters")
    
    # Test explorer
    explorer = FrontierExplorer(map_grid)
    goal = explorer.get_next_goal(robot_pose)
    
    if goal:
        print(f"\nNext exploration goal: ({goal[0]:.2f}, {goal[1]:.2f})")
    
    # Visualize
    print("\nGenerating visualization...")
    img = map_grid.get_occupancy_image(robot_pose)
    cv2.imwrite('/tmp/test_map.png', img)
    print("Map saved to /tmp/test_map.png")
    
    # Save map
    map_grid.save_map('/tmp/test_library_map')
    print("\n✓ Test complete!")