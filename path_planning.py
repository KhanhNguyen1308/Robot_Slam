"""
Path Planning Module
A* path planning for indoor navigation
"""
import numpy as np
import heapq
from typing import List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class AStarPlanner:
    """
    A* path planning algorithm
    Plans collision-free paths on occupancy grid
    """
    
    def __init__(self, occupancy_grid):
        """
        Args:
            occupancy_grid: OccupancyGrid instance
        """
        self.map = occupancy_grid
        self.path = []
        self.smoothed_path = []
        
    def plan(self, 
             start: Tuple[float, float],
             goal: Tuple[float, float],
             inflation_radius: int = 3) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal
        
        Args:
            start: (x, y) start position in world coordinates
            goal: (x, y) goal position in world coordinates
            inflation_radius: Obstacle inflation in grid cells (safety margin)
        
        Returns:
            List of (x, y) waypoints in world coordinates, or None if no path
        """
        # Convert to grid coordinates
        start_grid = self.map.world_to_grid(start[0], start[1])
        goal_grid = self.map.world_to_grid(goal[0], goal[1])
        
        # Check validity
        if not self.map.is_valid(start_grid[0], start_grid[1]):
            logger.error(f"Start position {start} out of bounds")
            return None
        
        if not self.map.is_valid(goal_grid[0], goal_grid[1]):
            logger.error(f"Goal position {goal} out of bounds")
            return None
        
        # Inflate obstacles
        inflated_grid = self._inflate_obstacles(inflation_radius)
        
        # Check if start/goal are in free space
        if inflated_grid[start_grid[1], start_grid[0]] > 0.5:
            logger.warning("Start position in obstacle")
            # Try to find nearest free cell
            start_grid = self._find_nearest_free(start_grid, inflated_grid)
            if start_grid is None:
                return None
        
        if inflated_grid[goal_grid[1], goal_grid[0]] > 0.5:
            logger.warning("Goal position in obstacle")
            # Try to find nearest free cell
            goal_grid = self._find_nearest_free(goal_grid, inflated_grid)
            if goal_grid is None:
                return None
        
        # Run A* algorithm
        path_grid = self._astar(start_grid, goal_grid, inflated_grid)
        
        if path_grid is None:
            logger.warning(f"No path found from {start} to {goal}")
            return None
        
        # Convert path to world coordinates
        path_world = [self.map.grid_to_world(x, y) for x, y in path_grid]
        
        # Smooth path
        smoothed_path = self._smooth_path(path_world, inflated_grid)
        
        self.path = path_world
        self.smoothed_path = smoothed_path
        
        logger.info(f"Path found: {len(path_grid)} waypoints -> {len(smoothed_path)} after smoothing")
        
        return smoothed_path
    
    def _inflate_obstacles(self, radius: int) -> np.ndarray:
        """Inflate obstacles for safety margin"""
        import cv2
        
        # Create binary obstacle map
        obstacle_map = (self.map.grid > 0.5).astype(np.uint8)
        
        # Dilate obstacles
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius*2+1, radius*2+1))
        inflated = cv2.dilate(obstacle_map, kernel, iterations=1)
        
        return inflated.astype(np.float32)
    
    def _find_nearest_free(self, 
                          pos: Tuple[int, int],
                          inflated_grid: np.ndarray,
                          max_search: int = 20) -> Optional[Tuple[int, int]]:
        """Find nearest free cell to given position"""
        x, y = pos
        
        for radius in range(1, max_search):
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    nx, ny = x + dx, y + dy
                    
                    if self.map.is_valid(nx, ny):
                        if inflated_grid[ny, nx] < 0.5:
                            return (nx, ny)
        
        return None
    
    def _astar(self,
               start: Tuple[int, int],
               goal: Tuple[int, int],
               inflated_grid: np.ndarray) -> Optional[List[Tuple[int, int]]]:
        """
        A* algorithm implementation
        
        Returns:
            List of (grid_x, grid_y) coordinates, or None if no path
        """
        # Priority queue: (f_score, counter, node)
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        # Track visited nodes
        came_from = {}
        
        # Cost from start
        g_score = {start: 0}
        
        # Estimated total cost
        f_score = {start: self._heuristic(start, goal)}
        
        # Closed set
        closed_set = set()
        
        # 8-connected grid
        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        neighbor_costs = [1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414]  # Diagonal vs straight
        
        while open_set:
            # Get node with lowest f_score
            _, _, current = heapq.heappop(open_set)
            
            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Explore neighbors
            for i, (dx, dy) in enumerate(neighbors):
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if not self.map.is_valid(neighbor[0], neighbor[1]):
                    continue
                
                # Check if occupied
                if inflated_grid[neighbor[1], neighbor[0]] > 0.5:
                    continue
                
                # Skip if already visited
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                tentative_g = g_score[current] + neighbor_costs[i]
                
                # Check if this path is better
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    # Add to open set
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1
        
        # No path found
        return None
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _reconstruct_path(self,
                         came_from: dict,
                         current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from map"""
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def _smooth_path(self,
                    path: List[Tuple[float, float]],
                    inflated_grid: np.ndarray,
                    max_iterations: int = 100) -> List[Tuple[float, float]]:
        """
        Smooth path using shortcutting
        
        Args:
            path: Original path in world coordinates
            inflated_grid: Inflated obstacle grid
            max_iterations: Maximum smoothing iterations
        
        Returns:
            Smoothed path
        """
        if len(path) <= 2:
            return path
        
        smoothed = path.copy()
        
        for _ in range(max_iterations):
            if len(smoothed) <= 2:
                break
            
            # Try to shortcut
            made_change = False
            
            i = 0
            while i < len(smoothed) - 2:
                # Try to connect point i directly to point i+2
                if self._is_line_free(smoothed[i], smoothed[i+2], inflated_grid):
                    # Remove intermediate point
                    smoothed.pop(i+1)
                    made_change = True
                else:
                    i += 1
            
            if not made_change:
                break
        
        return smoothed
    
    def _is_line_free(self,
                     p1: Tuple[float, float],
                     p2: Tuple[float, float],
                     inflated_grid: np.ndarray) -> bool:
        """Check if line between two points is collision-free"""
        # Convert to grid
        x1, y1 = self.map.world_to_grid(p1[0], p1[1])
        x2, y2 = self.map.world_to_grid(p2[0], p2[1])
        
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        x, y = x1, y1
        
        while True:
            # Check if current cell is free
            if not self.map.is_valid(x, y):
                return False
            
            if inflated_grid[y, x] > 0.5:
                return False
            
            if x == x2 and y == y2:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True


class PurePursuitController:
    """
    Pure Pursuit path following controller
    """
    
    def __init__(self, lookahead_distance: float = 0.5):
        """
        Args:
            lookahead_distance: Lookahead distance in meters
        """
        self.lookahead_distance = lookahead_distance
        self.current_waypoint_idx = 0
        
    def compute_velocity(self,
                        robot_pose: np.ndarray,
                        path: List[Tuple[float, float]],
                        max_linear: float = 0.3,
                        max_angular: float = 1.5) -> Tuple[float, float]:
        """
        Compute velocity commands to follow path
        
        Args:
            robot_pose: 4x4 transformation matrix
            path: List of waypoints
            max_linear: Maximum linear velocity
            max_angular: Maximum angular velocity
        
        Returns:
            (linear, angular) velocity commands
        """
        if len(path) == 0:
            return (0.0, 0.0)
        
        # Get robot position
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        robot_yaw = np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point(
            (robot_x, robot_y), path
        )
        
        if lookahead_point is None:
            # Reached end of path
            return (0.0, 0.0)
        
        # Calculate angle to lookahead point
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        target_angle = np.arctan2(dy, dx)
        
        # Angle difference
        angle_diff = self._normalize_angle(target_angle - robot_yaw)
        
        # Pure pursuit control law
        # Angular velocity proportional to angle difference
        angular = np.clip(2.0 * angle_diff, -max_angular, max_angular)
        
        # Linear velocity (slow down for sharp turns)
        if abs(angle_diff) > np.pi / 4:  # 45 degrees
            linear = max_linear * 0.3  # Slow down
        else:
            linear = max_linear
        
        return (linear, angular)
    
    def _find_lookahead_point(self,
                             robot_pos: Tuple[float, float],
                             path: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """Find lookahead point on path"""
        min_dist_to_path = float('inf')
        closest_idx = self.current_waypoint_idx
        
        # Find closest point on path
        for i in range(self.current_waypoint_idx, len(path)):
            dist = np.sqrt((path[i][0] - robot_pos[0])**2 + 
                          (path[i][1] - robot_pos[1])**2)
            
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_idx = i
        
        self.current_waypoint_idx = closest_idx
        
        # Find lookahead point
        for i in range(closest_idx, len(path)):
            dist = np.sqrt((path[i][0] - robot_pos[0])**2 + 
                          (path[i][1] - robot_pos[1])**2)
            
            if dist >= self.lookahead_distance:
                return path[i]
        
        # Return last point if no point at lookahead distance
        if len(path) > 0:
            return path[-1]
        
        return None
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def is_goal_reached(self,
                       robot_pose: np.ndarray,
                       goal: Tuple[float, float],
                       threshold: float = 0.2) -> bool:
        """Check if robot reached goal"""
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        
        dist = np.sqrt((robot_x - goal[0])**2 + (robot_y - goal[1])**2)
        
        return dist < threshold
    
    def reset(self):
        """Reset controller state"""
        self.current_waypoint_idx = 0


# Example usage
if __name__ == '__main__':
    from slam_mapping import OccupancyGrid
    
    logging.basicConfig(level=logging.INFO)
    
    print("Testing A* Path Planning")
    print("=" * 60)
    
    # Create test map
    map_grid = OccupancyGrid(width_meters=10.0, height_meters=10.0, resolution=0.1)
    
    # Add some obstacles
    for x in range(50, 70):
        for y in range(30, 80):
            map_grid.grid[y, x] = 1.0  # Wall
    
    # Plan path
    planner = AStarPlanner(map_grid)
    
    start = (0.0, 0.0)
    goal = (3.0, 3.0)
    
    print(f"Planning from {start} to {goal}...")
    
    path = planner.plan(start, goal, inflation_radius=2)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        print(f"  Start: {path[0]}")
        print(f"  Goal: {path[-1]}")
        
        # Visualize
        import cv2
        img = map_grid.get_occupancy_image(path=path)
        cv2.imwrite('/tmp/test_path.png', img)
        print("  Path visualization saved to /tmp/test_path.png")
    else:
        print("✗ No path found")
    
    print("\n✓ Test complete!")