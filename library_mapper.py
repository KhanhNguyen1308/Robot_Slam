"""
Autonomous Library Mapper
Complete system for autonomous exploration and mapping of indoor library environment
"""
import numpy as np
import cv2
import time
import logging
from typing import Optional, List, Tuple
from enum import Enum

from slam_mapping import OccupancyGrid, FrontierExplorer
from path_planning import AStarPlanner, PurePursuitController

logger = logging.getLogger(__name__)


class MappingState(Enum):
    """States for mapping state machine"""
    IDLE = 0
    EXPLORING = 1
    PLANNING = 2
    NAVIGATING = 3
    COMPLETED = 4
    ERROR = 5


class LibraryMapper:
    """
    Autonomous library mapping system
    Combines SLAM, exploration, and path planning
    """
    
    def __init__(self, 
                 map_width: float = 15.0,
                 map_height: float = 15.0,
                 resolution: float = 0.05):
        """
        Args:
            map_width: Map width in meters
            map_height: Map height in meters
            resolution: Grid resolution in meters
        """
        # Components
        self.occupancy_map = OccupancyGrid(map_width, map_height, resolution)
        self.explorer = FrontierExplorer(self.occupancy_map)
        self.planner = AStarPlanner(self.occupancy_map)
        self.controller = PurePursuitController(lookahead_distance=0.5)
        
        # State
        self.state = MappingState.IDLE
        self.current_goal = None
        self.current_path = None
        
        # Parameters
        self.max_linear_speed = 0.25  # m/s (slow for safety)
        self.max_angular_speed = 1.0  # rad/s
        
        # Statistics
        self.start_time = None
        self.total_distance_traveled = 0.0
        self.last_pose = None
        
        # Safety
        self.stuck_counter = 0
        self.max_stuck_count = 10
        
    def update(self,
               robot_pose: np.ndarray,
               depth_map: np.ndarray,
               camera_params: dict,
               obstacles: List = None) -> Tuple[float, float]:
        """
        Main update loop for autonomous mapping
        
        Args:
            robot_pose: 4x4 transformation matrix
            depth_map: Depth map in meters
            camera_params: Camera parameters
            obstacles: List of detected obstacles (optional)
        
        Returns:
            (linear, angular) velocity commands
        """
        # Update map from depth
        self.occupancy_map.update_from_depth(robot_pose, depth_map, camera_params)
        
        # Track distance
        if self.last_pose is not None:
            dx = robot_pose[0, 3] - self.last_pose[0, 3]
            dy = robot_pose[1, 3] - self.last_pose[1, 3]
            self.total_distance_traveled += np.sqrt(dx**2 + dy**2)
        self.last_pose = robot_pose.copy()
        
        # State machine
        if self.state == MappingState.IDLE:
            return self._state_idle()
        
        elif self.state == MappingState.EXPLORING:
            return self._state_exploring(robot_pose)
        
        elif self.state == MappingState.PLANNING:
            return self._state_planning(robot_pose)
        
        elif self.state == MappingState.NAVIGATING:
            return self._state_navigating(robot_pose, obstacles)
        
        elif self.state == MappingState.COMPLETED:
            return (0.0, 0.0)
        
        else:  # ERROR
            return (0.0, 0.0)
    
    def _state_idle(self) -> Tuple[float, float]:
        """Idle state - waiting to start"""
        return (0.0, 0.0)
    
    def _state_exploring(self, robot_pose: np.ndarray) -> Tuple[float, float]:
        """Finding next exploration goal"""
        logger.info("State: EXPLORING - Finding frontier...")
        
        # Get next exploration goal
        goal = self.explorer.get_next_goal(robot_pose)
        
        if goal is None:
            # Exploration complete
            logger.info("🎉 EXPLORATION COMPLETE!")
            self.state = MappingState.COMPLETED
            return (0.0, 0.0)
        
        self.current_goal = goal
        self.state = MappingState.PLANNING
        
        return (0.0, 0.0)
    
    def _state_planning(self, robot_pose: np.ndarray) -> Tuple[float, float]:
        """Planning path to goal"""
        logger.info(f"State: PLANNING - Planning to {self.current_goal}...")
        
        robot_x = robot_pose[0, 3]
        robot_y = robot_pose[1, 3]
        
        # Plan path
        path = self.planner.plan(
            start=(robot_x, robot_y),
            goal=self.current_goal,
            inflation_radius=4  # Safety margin (20cm with 5cm resolution)
        )
        
        if path is None:
            logger.warning("Failed to plan path to goal")
            # Try to find another goal
            self.state = MappingState.EXPLORING
            return (0.0, 0.0)
        
        self.current_path = path
        self.controller.reset()
        self.state = MappingState.NAVIGATING
        self.stuck_counter = 0
        
        logger.info(f"Path planned with {len(path)} waypoints")
        
        return (0.0, 0.0)
    
    def _state_navigating(self,
                         robot_pose: np.ndarray,
                         obstacles: List = None) -> Tuple[float, float]:
        """Following planned path"""
        if self.current_path is None or self.current_goal is None:
            self.state = MappingState.EXPLORING
            return (0.0, 0.0)
        
        # Check if goal reached
        if self.controller.is_goal_reached(robot_pose, self.current_goal, threshold=0.3):
            logger.info("✓ Goal reached!")
            self.state = MappingState.EXPLORING
            return (0.0, 0.0)
        
        # Emergency obstacle avoidance
        if obstacles and len(obstacles) > 0:
            # Check for very close obstacles
            closest = min(obstacles, key=lambda o: o.distance)
            
            if closest.distance < 0.35:  # 35cm emergency stop
                logger.warning(f"⚠ Emergency obstacle at {closest.distance:.2f}m")
                # Replan
                self.state = MappingState.PLANNING
                return (0.0, 0.0)
        
        # Compute velocity using pure pursuit
        linear, angular = self.controller.compute_velocity(
            robot_pose,
            self.current_path,
            max_linear=self.max_linear_speed,
            max_angular=self.max_angular_speed
        )
        
        # Check if stuck
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            self.stuck_counter += 1
            
            if self.stuck_counter > self.max_stuck_count:
                logger.warning("Robot appears stuck - replanning")
                self.state = MappingState.PLANNING
                self.stuck_counter = 0
                return (0.0, 0.0)
        else:
            self.stuck_counter = 0
        
        return (linear, angular)
    
    def start_mapping(self):
        """Start autonomous mapping"""
        logger.info("=" * 60)
        logger.info("STARTING AUTONOMOUS LIBRARY MAPPING")
        logger.info("=" * 60)
        
        self.state = MappingState.EXPLORING
        self.start_time = time.time()
        self.total_distance_traveled = 0.0
        
        logger.info(f"Map size: {self.occupancy_map.width}x{self.occupancy_map.height} cells")
        logger.info(f"Resolution: {self.occupancy_map.resolution}m/cell")
        logger.info(f"Coverage area: {self.occupancy_map.width_m}x{self.occupancy_map.height_m}m")
    
    def stop_mapping(self):
        """Stop autonomous mapping"""
        self.state = MappingState.IDLE
        
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        logger.info("=" * 60)
        logger.info("MAPPING STOPPED")
        logger.info("=" * 60)
        logger.info(f"Elapsed time: {elapsed:.1f}s")
        logger.info(f"Exploration: {self.occupancy_map.get_exploration_progress()*100:.1f}%")
        logger.info(f"Distance traveled: {self.total_distance_traveled:.2f}m")
        logger.info(f"Explored cells: {self.occupancy_map.explored_cells}")
    
    def get_status(self) -> dict:
        """Get mapping status"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        return {
            "state": self.state.name,
            "exploration_progress": self.occupancy_map.get_exploration_progress(),
            "explored_cells": self.occupancy_map.explored_cells,
            "total_cells": self.occupancy_map.total_cells,
            "current_goal": self.current_goal,
            "path_length": len(self.current_path) if self.current_path else 0,
            "elapsed_time": elapsed,
            "distance_traveled": self.total_distance_traveled,
            "map_complete": self.explorer.exploration_complete
        }
    
    def get_map_image(self, robot_pose: Optional[np.ndarray] = None) -> np.ndarray:
        """Get map visualization"""
        return self.occupancy_map.get_occupancy_image(
            robot_pose=robot_pose,
            path=self.current_path
        )
    
    def save_map(self, filename: str):
        """Save map to file"""
        self.occupancy_map.save_map(filename)
        
        # Also save metadata
        status = self.get_status()
        
        import json
        with open(filename + '_metadata.json', 'w') as f:
            # Convert numpy arrays to lists for JSON
            status_json = {k: (v.tolist() if isinstance(v, np.ndarray) else v) 
                          for k, v in status.items()}
            json.dump(status_json, f, indent=2)
        
        logger.info(f"Map and metadata saved to {filename}")
    
    def load_map(self, filename: str):
        """Load map from file"""
        self.occupancy_map.load_map(filename)
        self.explorer = FrontierExplorer(self.occupancy_map)
        logger.info(f"Map loaded from {filename}")


class LibraryMapVisualizer:
    """
    Real-time visualization for library mapping
    """
    
    def __init__(self, mapper: LibraryMapper):
        self.mapper = mapper
        self.window_name = "Library Map"
        
    def show(self, robot_pose: Optional[np.ndarray] = None):
        """Show map in window"""
        img = self.mapper.get_map_image(robot_pose)
        
        # Resize for better visibility
        scale = 2
        img = cv2.resize(img, (img.shape[1]*scale, img.shape[0]*scale), 
                        interpolation=cv2.INTER_NEAREST)
        
        # Add status overlay
        status = self.mapper.get_status()
        
        # Status panel
        panel_height = 120
        panel = np.zeros((panel_height, img.shape[1], 3), dtype=np.uint8)
        
        y = 25
        texts = [
            f"State: {status['state']}",
            f"Progress: {status['exploration_progress']*100:.1f}%",
            f"Time: {status['elapsed_time']:.0f}s",
            f"Distance: {status['distance_traveled']:.1f}m"
        ]
        
        for text in texts:
            cv2.putText(panel, text, (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y += 25
        
        # Combine
        combined = np.vstack([img, panel])
        
        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)
    
    def close(self):
        """Close visualization window"""
        cv2.destroyWindow(self.window_name)


# Example/Demo usage
if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print("=" * 60)
    print("AUTONOMOUS LIBRARY MAPPER - DEMO")
    print("=" * 60)
    
    # Create mapper
    mapper = LibraryMapper(
        map_width=10.0,
        map_height=10.0,
        resolution=0.05
    )
    
    # Start mapping
    mapper.start_mapping()
    
    # Simulate mapping loop
    robot_pose = np.eye(4)
    camera_params = {'focal_length': 700, 'width': 640, 'height': 480}
    
    for i in range(10):
        print(f"\n--- Iteration {i+1} ---")
        
        # Simulate depth map
        depth_map = np.ones((480, 640)) * 2.0
        depth_map[200:300, 250:350] = 0.8  # Obstacle
        
        # Update mapper
        linear, angular = mapper.update(robot_pose, depth_map, camera_params)
        
        print(f"Command: linear={linear:.2f}, angular={angular:.2f}")
        
        # Get status
        status = mapper.get_status()
        print(f"State: {status['state']}")
        print(f"Progress: {status['exploration_progress']*100:.1f}%")
        
        # Simulate robot movement
        robot_pose[0, 3] += linear * 0.1
        robot_pose[1, 3] += angular * 0.1
        
        time.sleep(0.1)
    
    # Stop mapping
    mapper.stop_mapping()
    
    # Save map
    mapper.save_map('/tmp/demo_library_map')
    
    print("\n✓ Demo complete!")