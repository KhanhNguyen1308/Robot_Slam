"""
Autonomous Room Mapping Controller
Integrates SLAM, motor control, and exploration strategy
"""
import numpy as np
import cv2
import logging
import time
import threading
import math
import heapq
from typing import Optional, Tuple, List, Dict
from collections import deque
from obstacle_detector import ObstacleDetector

logger = logging.getLogger(__name__)


class OccupancyGrid:
    """
    2D occupancy grid map for robot navigation
    """
    
    def __init__(self, 
                 width: float = 10.0,      # meters
                 height: float = 10.0,     # meters
                 resolution: float = 0.05): # meters per cell
        
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Grid dimensions
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        
        # Occupancy grid: 0 = free, 100 = occupied, -1 = unknown
        self.grid = np.ones((self.grid_height, self.grid_width), dtype=np.int8) * -1
        
        # Origin (center of grid)
        self.origin_x = width / 2.0
        self.origin_y = height / 2.0
        
        logger.info(f"Occupancy grid: {self.grid_width}x{self.grid_height} cells")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x + self.origin_x) / self.resolution)
        grid_y = int((y + self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = grid_x * self.resolution - self.origin_x
        y = grid_y * self.resolution - self.origin_y
        return x, y
    
    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid"""
        return (0 <= grid_x < self.grid_width and 
                0 <= grid_y < self.grid_height)
    
    def set_occupied(self, x: float, y: float):
        """Mark a world position as occupied"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if self.is_valid(grid_x, grid_y):
            self.grid[grid_y, grid_x] = 100
    
    def set_free(self, x: float, y: float):
        """Mark a world position as free"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if self.is_valid(grid_x, grid_y):
            self.grid[grid_y, grid_x] = 0
    
    def get_occupancy(self, x: float, y: float) -> int:
        """Get occupancy value at world position"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if self.is_valid(grid_x, grid_y):
            return self.grid[grid_y, grid_x]
        return -1
    
    def update_with_scan(self, robot_x: float, robot_y: float, robot_theta: float,
                        ranges: List[float], angles: List[float], max_range: float = 3.0):
        """
        Update grid with laser scan-like data
        
        Args:
            robot_x, robot_y: Robot position
            robot_theta: Robot orientation
            ranges: List of range measurements
            angles: List of angles for each range
            max_range: Maximum valid range
        """
        for angle, r in zip(angles, ranges):
            if r <= 0 or r > max_range:
                continue
            
            # Calculate endpoint in world frame
            global_angle = robot_theta + angle
            end_x = robot_x + r * np.cos(global_angle)
            end_y = robot_y + r * np.sin(global_angle)
            
            # Ray tracing (Bresenham-like)
            robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
            end_gx, end_gy = self.world_to_grid(end_x, end_y)
            
            # Mark cells along ray as free
            line_points = self._bresenham_line(robot_gx, robot_gy, end_gx, end_gy)
            for gx, gy in line_points[:-1]:
                if self.is_valid(gx, gy):
                    self.grid[gy, gx] = 0  # Free
            
            # Mark endpoint as occupied
            if self.is_valid(end_gx, end_gy):
                self.grid[end_gy, end_gx] = 100  # Occupied
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def get_frontiers(self) -> List[Tuple[int, int]]:
        """
        Find frontier cells (boundary between known free and unknown space).
        Uses vectorized numpy/cv2 operations instead of Python loops —
        ~100x faster on a 200x200 grid.

        Returns:
            List of (grid_x, grid_y) frontier cells
        """
        free    = (self.grid == 0).astype(np.uint8)
        unknown = (self.grid == -1).astype(np.uint8)

        # Dilate unknown mask by 1 pixel so every cell adjacent to unknown
        # space becomes an "unknown-neighbour"
        kernel = np.ones((3, 3), np.uint8)
        unknown_dilated = cv2.dilate(unknown, kernel)

        # Frontier = free AND touching unknown
        frontier_mask = free & unknown_dilated
        ys, xs = np.where(frontier_mask)
        return list(zip(xs.tolist(), ys.tolist()))
    
    def get_image(self) -> np.ndarray:
        """Get grid as image for visualization"""
        # Create RGB image
        img = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8)
        
        # Unknown = gray
        img[self.grid == -1] = [128, 128, 128]
        # Free = white
        img[self.grid == 0] = [255, 255, 255]
        # Occupied = black
        img[self.grid == 100] = [0, 0, 0]
        
        return img


class ExplorationPlanner:
    """
    Frontier-based exploration planner
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid, robot_radius: float = 0.2):
        self.grid = occupancy_grid
        self.current_goal = None
        self.goal_tolerance = 0.2       # metres
        self.robot_radius = robot_radius  # metres — used for obstacle inflation in A*
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        
    def find_nearest_frontier(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        """
        Find nearest frontier point
        
        Returns:
            (goal_x, goal_y) in world coordinates
        """
        frontiers = self.grid.get_frontiers()
        
        if not frontiers:
            logger.warning("No frontiers found")
            return None
        
        # Find nearest frontier
        robot_gx, robot_gy = self.grid.world_to_grid(robot_x, robot_y)
        
        min_dist = float('inf')
        nearest = None
        
        for fx, fy in frontiers:
            dist = np.sqrt((fx - robot_gx)**2 + (fy - robot_gy)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = (fx, fy)
        
        if nearest:
            goal_x, goal_y = self.grid.grid_to_world(nearest[0], nearest[1])
            return goal_x, goal_y
        
        return None
    
    def is_goal_reached(self, robot_x: float, robot_y: float) -> bool:
        """Check if current goal is reached"""
        if self.current_goal is None:
            return True
        
        goal_x, goal_y = self.current_goal
        dist = np.sqrt((robot_x - goal_x)**2 + (robot_y - goal_y)**2)
        
        return dist < self.goal_tolerance
    
    def plan_path(self, start: Tuple[float, float],
                  goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        A* path planning on the occupancy grid with obstacle inflation.

        Unknown cells (grid == -1) are traversable but carry an extra cost so
        the robot prefers known free space when available.

        Returns:
            Ordered list of (world_x, world_y) waypoints, or None if blocked.
        """
        # --- grid coordinates ---
        sx, sy = self.grid.world_to_grid(start[0], start[1])
        gx, gy = self.grid.world_to_grid(goal[0],  goal[1])
        sx = int(np.clip(sx, 0, self.grid.grid_width  - 1))
        sy = int(np.clip(sy, 0, self.grid.grid_height - 1))
        gx = int(np.clip(gx, 0, self.grid.grid_width  - 1))
        gy = int(np.clip(gy, 0, self.grid.grid_height - 1))

        # --- inflate occupied cells by robot radius ---
        inflation_px = max(1, int(self.robot_radius / self.grid.resolution))
        occupied = (self.grid.grid == 100).astype(np.uint8)
        kernel   = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * inflation_px + 1, 2 * inflation_px + 1)
        )
        inflated = cv2.dilate(occupied, kernel).astype(bool)

        # --- A* search (8-connected) ---
        open_set: List = []   # heap entries: (f, g, x, y)
        heapq.heappush(open_set, (0.0, 0.0, sx, sy))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_cost:    Dict[Tuple[int, int], float]           = {(sx, sy): 0.0}

        def heuristic(x: int, y: int) -> float:
            # Chebyshev distance — consistent with 8-connected movement
            return float(max(abs(x - gx), abs(y - gy)))

        while open_set:
            _, g, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):          # goal reached — reconstruct path
                path: List[Tuple[float, float]] = []
                node: Tuple[int, int] = (cx, cy)
                while node in came_from:
                    wx, wy = self.grid.grid_to_world(node[0], node[1])
                    path.append((wx, wy))
                    node = came_from[node]
                wx, wy = self.grid.grid_to_world(sx, sy)
                path.append((wx, wy))
                path.reverse()
                return path

            if g > g_cost.get((cx, cy), float('inf')):
                continue  # stale heap entry

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = cx + dx, cy + dy
                if not self.grid.is_valid(nx, ny):
                    continue
                if inflated[ny, nx]:
                    continue  # inside obstacle inflation zone
                extra = 2.0 if self.grid.grid[ny, nx] == -1 else 0.0
                new_g = g + math.sqrt(dx * dx + dy * dy) + extra
                if new_g < g_cost.get((nx, ny), float('inf')):
                    g_cost[(nx, ny)] = new_g
                    heapq.heappush(open_set, (new_g + heuristic(nx, ny), new_g, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

        logger.warning(f"A* found no path from {start} to {goal}")
        return None


class AutonomousMapper:
    """
    Main autonomous mapping controller
    Coordinates SLAM, mapping, and exploration
    """
    
    def __init__(self,
                 motor_controller,
                 slam_system,
                 occupancy_grid: OccupancyGrid,
                 obstacle_detector: Optional[ObstacleDetector] = None,
                 max_linear_speed: float = 0.2,
                 max_angular_speed: float = 1.0,
                 camera_matrix: Optional[np.ndarray] = None,
                 baseline: float = 0.14,
                 robot_radius: float = 0.2):

        self.motor = motor_controller
        self.slam = slam_system
        self.grid = occupancy_grid
        self.camera_matrix = camera_matrix  # for correct depth in update_map_from_stereo
        self.baseline = baseline            # stereo baseline in metres
        self.planner = ExplorationPlanner(occupancy_grid, robot_radius=robot_radius)
        
        # Obstacle detection
        self.obstacle_detector = obstacle_detector or ObstacleDetector()
        self.last_obstacle_detection = None
        self.obstacle_avoidance_enabled = True
        
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # State
        self.running = False
        self.mapping_mode = "exploration"  # exploration, manual, idle
        self.thread = None
        
        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Statistics
        self.total_distance = 0.0
        self.exploration_time = 0.0
        self.frontiers_visited = 0
        self.obstacles_avoided = 0
        
        logger.info("Autonomous mapper initialized with obstacle avoidance")
    
    def start_exploration(self):
        """Start autonomous exploration"""
        if not self.running:
            self.running = True
            self.mapping_mode = "exploration"
            self.thread = threading.Thread(target=self._exploration_loop, daemon=True)
            self.thread.start()
            logger.info("Autonomous exploration started")
    
    def stop_exploration(self):
        """Stop autonomous exploration"""
        self.running = False
        self.mapping_mode = "idle"
        
        # Don't join if we're in the exploration thread itself
        if self.thread and threading.current_thread() != self.thread:
            self.thread.join(timeout=2.0)
        
        self.motor.stop()
        logger.info("Autonomous exploration stopped")
    
    def _exploration_loop(self):
        """Main exploration control loop"""
        start_time = time.time()
        
        while self.running:
            try:
                # Get current pose from SLAM (includes IMU fusion if available)
                slam_pose = self.slam.get_2d_pose()
                if slam_pose:
                    new_x, new_y, new_theta = slam_pose
                    
                    # Validate orientation change (detect jumps)
                    if self.theta is not None:
                        theta_diff = abs(new_theta - self.theta)
                        # Normalize to [-pi, pi]
                        if theta_diff > np.pi:
                            theta_diff = 2 * np.pi - theta_diff
                        
                        # If orientation jumps too much (>60 degrees in one frame)
                        # and we're not moving much, it's likely a visual tracking error
                        if theta_diff > np.pi/3:  # 60 degrees
                            dist_moved = np.sqrt((new_x - self.x)**2 + (new_y - self.y)**2)
                            if dist_moved < 0.05:  # Less than 5cm movement
                                logger.warning(f"Large orientation jump detected ({np.degrees(theta_diff):.1f}°), smoothing...")
                                # Smooth the orientation change - allow max 10 degrees per frame
                                max_change = np.pi/18  # 10 degrees
                                sign = 1 if (new_theta - self.theta) > 0 else -1
                                new_theta = self.theta + sign * min(theta_diff, max_change)
                    
                    self.x, self.y, self.theta = new_x, new_y, new_theta
                
                # Advance to next path waypoint when current one is reached
                if (self.planner.current_path and
                        self.planner.path_index < len(self.planner.current_path) - 1):
                    wp = self.planner.current_path[self.planner.path_index]
                    wp_dist = math.sqrt((self.x - wp[0])**2 + (self.y - wp[1])**2)
                    if wp_dist < self.planner.goal_tolerance:
                        self.planner.path_index += 1
                        logger.debug(
                            f"Waypoint reached, advancing to index {self.planner.path_index}"
                        )

                # Check if the FINAL goal is reached — find the next frontier
                if self.planner.is_goal_reached(self.x, self.y):
                    goal = self.planner.find_nearest_frontier(self.x, self.y)

                    if goal is None:
                        logger.info("Exploration complete - no more frontiers")
                        self.stop_exploration()
                        break

                    self.planner.current_goal = goal
                    self.frontiers_visited += 1
                    logger.info(f"New frontier goal: ({goal[0]:.2f}, {goal[1]:.2f})")

                    # Plan A* path to the new frontier
                    path = self.planner.plan_path((self.x, self.y), goal)
                    if path and len(path) > 1:
                        self.planner.current_path = path
                        self.planner.path_index = 1  # index 0 is current position
                        logger.info(f"A* path planned: {len(path)} waypoints")
                    else:
                        # No path found — navigate directly
                        self.planner.current_path = [goal]
                        self.planner.path_index = 0
                        logger.warning("No A* path found, navigating directly to goal")

                # Navigate towards current waypoint (or final goal as fallback)
                if (self.planner.current_path and
                        self.planner.path_index < len(self.planner.current_path)):
                    self._navigate_to_goal(
                        self.planner.current_path[self.planner.path_index]
                    )
                elif self.planner.current_goal:
                    self._navigate_to_goal(self.planner.current_goal)
                
                time.sleep(0.1)  # 10 Hz control loop
                
            except Exception as e:
                logger.error(f"Exploration loop error: {e}")
                time.sleep(1.0)
        
        self.exploration_time = time.time() - start_time
    
    def _navigate_to_goal(self, goal: Tuple[float, float]):
        """
        Simple proportional controller to navigate to goal with obstacle avoidance
        
        Args:
            goal: (x, y) target position
        """
        goal_x, goal_y = goal
        
        # Calculate error
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Calculate desired heading
        desired_theta = np.arctan2(dy, dx)
        angle_error = self._normalize_angle(desired_theta - self.theta)
        
        # Proportional control
        Kp_linear = 0.5
        Kp_angular = 2.0
        
        # DEADBAND: Don't adjust if angle error is small (prevents oscillation)
        angle_deadband = 0.15  # ~8.6 degrees - ignore small errors
        if abs(angle_error) < angle_deadband:
            angle_error = 0.0  # Within acceptable range, no rotation needed
        
        # Calculate desired velocities
        linear_vel = Kp_linear * distance
        angular_vel = Kp_angular * angle_error
        
        # Minimum angular velocity threshold to avoid micro-adjustments
        min_angular_threshold = 0.1  # rad/s
        if 0 < abs(angular_vel) < min_angular_threshold:
            angular_vel = 0.0  # Too small to be useful, set to zero
        
        # Limit velocities
        linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # Reduce linear speed when turning (with larger threshold to avoid stopping unnecessarily)
        if abs(angle_error) > 0.7:  # ~40 degrees - only slow down for significant turns
            linear_vel *= 0.5
        
        # Apply obstacle avoidance if enabled and detection available
        if self.obstacle_avoidance_enabled and self.last_obstacle_detection:
            safe_linear, safe_angular = self.obstacle_detector.get_safety_velocity(
                linear_vel, angular_vel, self.last_obstacle_detection
            )
            
            # Check if we had to adjust for obstacles
            if abs(safe_linear - linear_vel) > 0.01 or abs(safe_angular - angular_vel) > 0.01:
                self.obstacles_avoided += 1
                logger.debug(f"Obstacle avoidance: adjusted velocity from "
                           f"({linear_vel:.2f}, {angular_vel:.2f}) to "
                           f"({safe_linear:.2f}, {safe_angular:.2f})")
            
            linear_vel = safe_linear
            angular_vel = safe_angular
        
        # Send command to motors
        self.motor.send_velocity(linear_vel, angular_vel)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def update_obstacle_detection(self,
                                  disparity: np.ndarray,
                                  camera_matrix: np.ndarray,
                                  baseline: float) -> Optional[Dict]:
        """
        Update obstacle detection from stereo disparity
        
        Args:
            disparity: Disparity map
            camera_matrix: Camera intrinsic matrix
            baseline: Stereo baseline in meters
            
        Returns:
            Detection result dict
        """
        if not self.obstacle_avoidance_enabled or disparity is None:
            return None
        
        # Get current velocity
        current_velocity = (self.motor.current_linear, self.motor.current_angular)
        
        # Detect obstacles
        detection_result = self.obstacle_detector.detect_from_disparity(
            disparity, camera_matrix, baseline, current_velocity
        )
        
        # Store for use in navigation
        self.last_obstacle_detection = detection_result
        
        # Log warnings/emergencies
        action = detection_result.get('action', {})
        if action.get('severity') in ['warning', 'critical']:
            logger.warning(action.get('message', 'Obstacle detected'))
        
        return detection_result
    
    def update_map_from_stereo(self, disparity: np.ndarray, camera_pose: np.ndarray):
        """
        Update occupancy grid from stereo disparity map
        
        Args:
            disparity: Disparity map
            camera_pose: Current camera pose (4x4 matrix)
        """
        # Extract obstacles from disparity
        # This is a simplified version - would need proper point cloud processing
        
        # Create pseudo laser scan from disparity
        height, width = disparity.shape
        ranges = []
        angles = []

        # Derive focal length and principal point from stored camera matrix
        if self.camera_matrix is not None:
            fx           = float(self.camera_matrix[0, 0])
            cx_principal = float(self.camera_matrix[0, 2])
        else:
            fx           = 500.0          # generic fallback
            cx_principal = width / 2.0

        # Sample horizontal band around image centre
        cy = height // 2
        for cx in range(0, width, 10):  # every 10 pixels
            d = float(disparity[cy, cx])
            if d > 0:
                # Correct stereo depth formula: depth = (fx * baseline) / disparity
                depth = (fx * self.baseline) / (d + 1e-6)
                if 0.1 < depth < 3.0:  # valid indoor range
                    # Horizontal angle from the principal point
                    angle = math.atan2(cx - cx_principal, fx)
                    ranges.append(depth)
                    angles.append(angle)
        
        # Update grid
        if ranges:
            self.grid.update_with_scan(self.x, self.y, self.theta, ranges, angles)
    
    def get_stats(self) -> dict:
        """Get mapping statistics"""
        # Count explored cells
        explored = np.sum(self.grid.grid >= 0)
        total_cells = self.grid.grid.size
        coverage = (explored / total_cells) * 100
        
        stats = {
            "running": self.running,
            "mode": self.mapping_mode,
            "position": (self.x, self.y, self.theta),
            "current_goal": self.planner.current_goal,
            "frontiers_visited": self.frontiers_visited,
            "exploration_time": self.exploration_time,
            "coverage_percent": coverage,
            "total_distance": self.total_distance,
            "obstacles_avoided": self.obstacles_avoided,
            "obstacle_avoidance_enabled": self.obstacle_avoidance_enabled
        }
        
        # Add obstacle detector stats if available
        if self.obstacle_detector:
            stats["obstacle_stats"] = self.obstacle_detector.get_stats()
        
        return stats
    
    def save_map(self, filepath: str):
        """Save occupancy grid to file"""
        np.savez(filepath, 
                 grid=self.grid.grid,
                 resolution=self.grid.resolution,
                 width=self.grid.width,
                 height=self.grid.height,
                 origin_x=self.grid.origin_x,
                 origin_y=self.grid.origin_y)
        logger.info(f"Map saved to {filepath}")
    
    def load_map(self, filepath: str):
        """Load occupancy grid from file"""
        data = np.load(filepath)
        self.grid.grid = data['grid']
        self.grid.resolution = float(data['resolution'])
        self.grid.width = float(data['width'])
        self.grid.height = float(data['height'])
        self.grid.origin_x = float(data['origin_x'])
        self.grid.origin_y = float(data['origin_y'])
        logger.info(f"Map loaded from {filepath}")