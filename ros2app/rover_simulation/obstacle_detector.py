
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math


class ObstacleDetector(Node):
    """Detect obstacles from LiDAR and publish occupancy grid."""

    def __init__(self):
        """Initialize obstacle detector node."""
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('obstacle_distance_threshold', 2.0)
        self.declare_parameter('min_obstacle_points', 5)
        
        self.obstacle_threshold = self.get_parameter(
            'obstacle_distance_threshold').value
        self.min_points = self.get_parameter('min_obstacle_points').value
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid, 'obstacle_map', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Grid configuration (matches rover_controller)
        self.grid_width = 100
        self.grid_height = 100
        self.grid_resolution = 0.1
        self.grid_origin = (-5.0, -5.0)
        
        self.get_logger().info('Obstacle detector initialized')

    def scan_callback(self, msg: LaserScan) -> None:
        """Process LiDAR scan data.
        
        Args:
            msg: LaserScan message
        """
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.grid_resolution
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid.info.origin.position.x = self.grid_origin[0]
        grid.info.origin.position.y = self.grid_origin[1]
        grid.info.origin.position.z = 0.0
        
        # Initialize grid with unknown values
        grid.data = [0] * (self.grid_width * self.grid_height)
        
        # Process scan data
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if (msg.range_min < range_val < msg.range_max and
                    range_val < self.obstacle_threshold):
                
                # Convert polar to Cartesian
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                # Convert to grid coordinates
                grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
                grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
                
                # Mark cell as occupied
                if (0 <= grid_x < self.grid_width and
                        0 <= grid_y < self.grid_height):
                    idx = grid_y * self.grid_width + grid_x
                    grid.data[idx] = 100  # Occupied
                    
                    # Inflate obstacles slightly
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = grid_x + dx, grid_y + dy
                            if (0 <= nx < self.grid_width and
                                    0 <= ny < self.grid_height):
                                nidx = ny * self.grid_width + nx
                                if grid.data[nidx] < 100:
                                    grid.data[nidx] = 75
            
            angle += msg.angle_increment
        
        self.map_pub.publish(grid)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()