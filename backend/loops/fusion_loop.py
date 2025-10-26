"""
Sensor Fusion Loop (runs at 2 Hz)
Fuses LiDAR point cloud + camera frames to create:
1. Occupancy grid for D* Lite pathfinding
2. Aligned depth map for YOLO object detection
3. Combined sensor data for navigation

Uses OpenCV for camera-LiDAR alignment and grid generation.
"""

import asyncio
import logging
import time
from typing import Optional, Tuple, List
import numpy as np
import cv2

logger = logging.getLogger(__name__)


class FusionLoop:
    """
    Sensor fusion loop that combines LiDAR and camera data.
    Generates occupancy grid for pathfinding and depth maps for detection.
    """

    def __init__(
        self,
        state_manager,
        grid_resolution: float = 0.1,  # meters per grid cell
        grid_size_x: int = 100,  # 10m x 10m grid
        grid_size_y: int = 100,
        max_obstacle_height: float = 0.5,  # meters - obstacles above this are ignored
        min_obstacle_height: float = 0.05,  # meters - obstacles below this are ignored
        obstacle_threshold: int = 5,  # minimum points to mark cell as obstacle
        interval_ms: int = 500  # 2 Hz
    ):
        """
        Initialize fusion loop.

        Args:
            state_manager: StateManager instance
            grid_resolution: Size of each grid cell in meters
            grid_size_x: Grid width in cells
            grid_size_y: Grid height in cells
            max_obstacle_height: Maximum height to consider as obstacle (ignore ceiling)
            min_obstacle_height: Minimum height to consider as obstacle (ignore ground)
            obstacle_threshold: Minimum points in cell to mark as obstacle
            interval_ms: Loop interval in milliseconds
        """
        self.state_manager = state_manager
        self.interval_seconds = interval_ms / 1000.0
        self.running = False

        # Grid parameters
        self.grid_resolution = grid_resolution
        self.grid_size_x = grid_size_x
        self.grid_size_y = grid_size_y
        self.max_obstacle_height = max_obstacle_height
        self.min_obstacle_height = min_obstacle_height
        self.obstacle_threshold = obstacle_threshold

        # Camera parameters (will be calibrated if available)
        self.camera_fov_horizontal = 60.0  # degrees
        self.camera_fov_vertical = 45.0  # degrees
        self.camera_width = 640
        self.camera_height = 480

        # Statistics
        self.total_fusions = 0
        self.total_obstacles_mapped = 0
        self.last_fusion_time = 0
        self.fusion_duration_avg = 0

        logger.info(
            f"Fusion loop initialized: "
            f"{grid_size_x}x{grid_size_y} grid, {grid_resolution}m resolution, "
            f"{interval_ms}ms interval"
        )

    async def run(self):
        """Main fusion loop - runs continuously."""
        self.running = True
        logger.info("Fusion loop started")

        while self.running:
            try:
                start_time = time.time()

                # Check if we have fresh sensor data
                lidar_data = self.state_manager.last_lidar_data
                camera_frame = self.state_manager.last_camera_frame

                if lidar_data is None:
                    logger.debug("No LiDAR data available, skipping fusion")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Check data freshness
                lidar_age = time.time() - self.state_manager.last_lidar_timestamp
                if lidar_age > 2.0:  # 2 second timeout
                    logger.warning(f"LiDAR data is stale ({lidar_age:.1f}s old)")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # STEP 1: Generate occupancy grid from LiDAR point cloud
                occupancy_grid, obstacle_count = self._generate_occupancy_grid(lidar_data)

                # STEP 2: Update state manager with occupancy grid
                self.state_manager.update_occupancy_grid(
                    occupancy_grid,
                    timestamp=time.time()
                )

                # STEP 3: If camera frame available, create aligned depth map
                depth_map = None
                if camera_frame is not None:
                    camera_age = time.time() - self.state_manager.last_camera_timestamp
                    if camera_age < 2.0:  # Only fuse if camera data is fresh
                        depth_map = self._create_depth_map(lidar_data, camera_frame)

                # STEP 4: Store fused data
                fused_data = {
                    "occupancy_grid": occupancy_grid,
                    "depth_map": depth_map,
                    "obstacle_count": obstacle_count,
                    "grid_resolution": self.grid_resolution,
                    "timestamp": time.time()
                }
                self.state_manager.update_fused_data(fused_data, time.time())

                # Update statistics
                self.total_fusions += 1
                self.total_obstacles_mapped += obstacle_count
                self.last_fusion_time = time.time()

                fusion_duration = time.time() - start_time
                self.fusion_duration_avg = (
                    (self.fusion_duration_avg * (self.total_fusions - 1) + fusion_duration)
                    / self.total_fusions
                )

                logger.info(
                    f"Fusion complete: {obstacle_count} obstacles, "
                    f"{fusion_duration*1000:.1f}ms (avg: {self.fusion_duration_avg*1000:.1f}ms)"
                )

            except Exception as e:
                logger.error(f"Error in fusion loop: {e}", exc_info=True)

            # Wait for next cycle
            await asyncio.sleep(self.interval_seconds)

        logger.info("Fusion loop stopped")

    def _generate_occupancy_grid(
        self, lidar_data: dict
    ) -> Tuple[np.ndarray, int]:
        """
        Generate 2D occupancy grid from LiDAR 3D point cloud.

        Args:
            lidar_data: LiDAR data dict with 'points' list

        Returns:
            Tuple of (occupancy_grid, obstacle_count)
            - occupancy_grid: 2D numpy array (0=free, 255=obstacle)
            - obstacle_count: Number of obstacles detected
        """
        # Initialize empty grid (0 = free space)
        grid = np.zeros((self.grid_size_y, self.grid_size_x), dtype=np.uint8)

        # Get robot's current position (grid center)
        from tools.odometry import odometry
        robot_pos = odometry.get_position()  # (x, y, heading)
        robot_x, robot_y = robot_pos[0], robot_pos[1]

        # Create accumulator for counting points per cell
        point_counts = np.zeros((self.grid_size_y, self.grid_size_x), dtype=np.int32)

        points = lidar_data.get('points', [])
        if not points:
            logger.debug("No LiDAR points available")
            return grid, 0

        # Process each LiDAR point
        for point in points:
            x = point.get('x', 0)  # meters in robot frame
            y = point.get('y', 0)
            z = point.get('z', 0)

            # Filter by height (ignore ground and ceiling)
            if z < self.min_obstacle_height or z > self.max_obstacle_height:
                continue

            # Convert to world coordinates
            world_x = robot_x + x
            world_y = robot_y + y

            # Convert to grid coordinates (robot is at center of grid)
            grid_x = int((world_x - (robot_x - self.grid_size_x * self.grid_resolution / 2)) / self.grid_resolution)
            grid_y = int((world_y - (robot_y - self.grid_size_y * self.grid_resolution / 2)) / self.grid_resolution)

            # Check bounds
            if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
                point_counts[grid_y, grid_x] += 1

        # Mark cells with enough points as obstacles
        obstacle_mask = point_counts >= self.obstacle_threshold
        grid[obstacle_mask] = 255

        obstacle_count = np.sum(obstacle_mask)

        logger.debug(
            f"Grid generated: {len(points)} points → {obstacle_count} obstacle cells"
        )

        return grid, obstacle_count

    def _create_depth_map(
        self, lidar_data: dict, camera_frame: bytes
    ) -> Optional[np.ndarray]:
        """
        Create aligned depth map by projecting LiDAR points onto camera frame.

        Uses OpenCV for camera projection and interpolation.

        Args:
            lidar_data: LiDAR point cloud data
            camera_frame: Raw camera JPEG bytes

        Returns:
            Depth map as 2D numpy array (same size as camera frame) or None
        """
        try:
            # Decode camera frame
            if isinstance(camera_frame, bytes):
                nparr = np.frombuffer(camera_frame, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is None:
                    logger.warning("Failed to decode camera frame")
                    return None
                h, w = img.shape[:2]
            else:
                # Already decoded
                img = camera_frame
                h, w = img.shape[:2]

            # Initialize depth map
            depth_map = np.zeros((h, w), dtype=np.float32)
            point_count_map = np.zeros((h, w), dtype=np.int32)

            # Get LiDAR points
            points = lidar_data.get('points', [])
            if not points:
                return None

            # Project each 3D LiDAR point to 2D camera pixel
            for point in points:
                x = point.get('x', 0)  # horizontal (left/right)
                y = point.get('y', 0)  # vertical (up/down)
                z = point.get('z', 0)  # depth (forward)

                if z <= 0 or z > 10:  # Filter invalid depths
                    continue

                # Simple pinhole camera projection
                # Assuming LiDAR and camera are roughly co-located
                # Convert 3D point to pixel coordinates using FOV
                pixel_x = int(w / 2 + (x / z) * (w / 2) / np.tan(np.radians(self.camera_fov_horizontal / 2)))
                pixel_y = int(h / 2 + (y / z) * (h / 2) / np.tan(np.radians(self.camera_fov_vertical / 2)))

                # Check bounds
                if 0 <= pixel_x < w and 0 <= pixel_y < h:
                    depth_map[pixel_y, pixel_x] += z
                    point_count_map[pixel_y, pixel_x] += 1

            # Average depths where multiple points hit same pixel
            valid_mask = point_count_map > 0
            depth_map[valid_mask] /= point_count_map[valid_mask]

            # Interpolate to fill gaps using OpenCV inpainting
            # Create mask for invalid pixels
            mask = (depth_map == 0).astype(np.uint8)

            # Convert depth map to 8-bit for inpainting
            depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_map_8bit = depth_map_normalized.astype(np.uint8)

            # Inpaint missing values
            if np.sum(mask) > 0 and np.sum(mask) < mask.size:
                depth_map_inpainted = cv2.inpaint(depth_map_8bit, mask, 3, cv2.INPAINT_TELEA)
                # Convert back to meters
                depth_map = depth_map_inpainted.astype(np.float32) * 10.0 / 255.0
            else:
                # Convert normalized map back to meters
                depth_map = depth_map_normalized * 10.0 / 255.0

            logger.debug(f"Depth map created: {np.sum(valid_mask)} valid pixels")

            return depth_map

        except Exception as e:
            logger.error(f"Error creating depth map: {e}")
            return None

    def get_stats(self) -> dict:
        """Get fusion loop statistics."""
        return {
            "running": self.running,
            "total_fusions": self.total_fusions,
            "total_obstacles_mapped": self.total_obstacles_mapped,
            "last_fusion_time": self.last_fusion_time,
            "fusion_duration_avg_ms": self.fusion_duration_avg * 1000,
            "grid_size": f"{self.grid_size_x}x{self.grid_size_y}",
            "grid_resolution": f"{self.grid_resolution}m"
        }
