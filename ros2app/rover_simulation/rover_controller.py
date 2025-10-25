"""
Rover controller node with D* Lite path planning.
PEP 8 compliant.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import math
import json
from typing import Tuple, Optional
from .dstar_lite import DStarLite


class RoverController(Node):
    """Main rover controller with path planning."""

    def __init__(self):
        """Initialize the rover controller node."""
        super().__init__('rover_controller')
        
        # Parameters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('planning_frequency', 2.0)
        
        self.max_linear_vel = self.get_parameter(
            'max_linear_velocity').value
        self.max_angular_vel = self.get_parameter(
            'max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        planning_freq = self.get_parameter('planning_frequency').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'rover_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, 'obstacle_map', self.obstacle_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # State variables
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.goal_pose: Optional[Tuple[float, float]] = None
        self.planner: Optional[DStarLite] = None
        self.path: Optional[list] = None
        self.path_index = 0
        
        # Grid configuration (10m x 10m with 0.1m resolution)
        self.grid_size = (100, 100)
        self.grid_resolution = 0.1
        self.grid_origin = (-5.0, -5.0)
        
        # Timer for control loop
        self.create_timer(1.0 / planning_freq, self.control_loop)
        
        self.get_logger().info('Rover controller initialized')

    def odom_callback(self, msg: Odometry) -> None:
        """Process odometry data.
        
        Args:
            msg: Odometry message
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z +
                        orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (position.x, position.y, yaw)

    def obstacle_callback(self, msg: OccupancyGrid) -> None:
        """Process obstacle map from obstacle detector.
        
        Args:
            msg: OccupancyGrid message with obstacle data
        """
        if self.planner is None or self.goal_pose is None:
            return
        
        obstacles = set()
        for i, value in enumerate(msg.data):
            if value > 50:  # Occupied cell
                grid_x = i % msg.info.width
                grid_y = i // msg.info.width
                obstacles.add((grid_x, grid_y))
        
        self.planner.update_obstacles(obstacles)
        self.path = self.planner.get_path()
        self.path_index = 0

    def goal_callback(self, msg: PoseStamped) -> None:
        """Set new goal position.
        
        Args:
            msg: PoseStamped message with goal
        """
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.goal_pose = (goal_x, goal_y)
        
        if self.current_pose is None:
            self.get_logger().warn('No current pose available')
            return
        
        start_grid = self._world_to_grid(self.current_pose[0],
                                         self.current_pose[1])
        goal_grid = self._world_to_grid(goal_x, goal_y)
        
        self.planner = DStarLite(self.grid_size, start_grid, goal_grid)
        self.path = self.planner.get_path()
        self.path_index = 0
        
        self.get_logger().info(f'New goal set: ({goal_x:.2f}, {goal_y:.2f})')

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates.
        
        Args:
            x: World x coordinate
            y: World y coordinate
            
        Returns:
            Grid coordinates (grid_x, grid_y)
        """
        grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
        return (grid_x, grid_y)

    def _grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates.
        
        Args:
            grid_x: Grid x coordinate
            grid_y: Grid y coordinate
            
        Returns:
            World coordinates (x, y)
        """
        x = grid_x * self.grid_resolution + self.grid_origin[0]
        y = grid_y * self.grid_resolution + self.grid_origin[1]
        return (x, y)

    def control_loop(self) -> None:
        """Main control loop for rover movement."""
        if (self.current_pose is None or self.goal_pose is None or
                self.path is None or len(self.path) == 0):
            self._stop_rover()
            return
        
        # Check if goal reached
        dist_to_goal = math.sqrt(
            (self.current_pose[0] - self.goal_pose[0])**2 +
            (self.current_pose[1] - self.goal_pose[1])**2
        )
        
        if dist_to_goal < self.goal_tolerance:
            self._stop_rover()
            self._publish_status('Goal reached')
            self.get_logger().info('Goal reached!')
            return
        
        # Get next waypoint
        if self.path_index >= len(self.path):
            self._stop_rover()
            return
        
        target_grid = self.path[self.path_index]
        target_world = self._grid_to_world(target_grid[0], target_grid[1])
        
        # Check if current waypoint reached
        dist_to_waypoint = math.sqrt(
            (self.current_pose[0] - target_world[0])**2 +
            (self.current_pose[1] - target_world[1])**2
        )
        
        if dist_to_waypoint < self.grid_resolution * 2:
            self.path_index += 1
            if self.path_index < len(self.path):
                target_grid = self.path[self.path_index]
                target_world = self._grid_to_world(target_grid[0],
                                                   target_grid[1])
        
        # Calculate control commands
        angle_to_target = math.atan2(
            target_world[1] - self.current_pose[1],
            target_world[0] - self.current_pose[0]
        )
        
        angle_diff = angle_to_target - self.current_pose[2]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd = Twist()
        
        if abs(angle_diff) > 0.2:
            cmd.angular.z = max(min(angle_diff * 2.0, self.max_angular_vel),
                               -self.max_angular_vel)
            cmd.linear.x = self.max_linear_vel * 0.3
        else:
            cmd.linear.x = self.max_linear_vel
            cmd.angular.z = max(min(angle_diff, self.max_angular_vel),
                               -self.max_angular_vel)
        
        self.cmd_vel_pub.publish(cmd)
        self._publish_status(f'Moving to waypoint {self.path_index}/'
                           f'{len(self.path)}')

    def _stop_rover(self) -> None:
        """Stop the rover."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _publish_status(self, status: str) -> None:
        """Publish rover status.
        
        Args:
            status: Status message
        """
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'position': self.current_pose[:2] if self.current_pose else None,
            'goal': self.goal_pose
        })
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = RoverController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()