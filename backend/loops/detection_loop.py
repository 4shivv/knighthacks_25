"""
Object Detection Loop (runs at 3-5 second intervals)
Processes camera frames with YOLO and stores detections in state manager.
"""

import asyncio
import logging
import time
from typing import Optional
import numpy as np
import cv2
from state_manager import DetectedObject
from config import settings

logger = logging.getLogger(__name__)


class DetectionLoop:
    """
    Detection loop that runs YOLO on camera frames periodically.
    """

    def __init__(
        self,
        state_manager,
        yolo_model,
        interval_ms: int = 3000
    ):
        """
        Initialize detection loop.

        Args:
            state_manager: StateManager instance
            yolo_model: YOLOModel instance
            interval_ms: Loop interval in milliseconds (default 3000ms = 3s)
        """
        self.state_manager = state_manager
        self.yolo_model = yolo_model
        self.interval_seconds = interval_ms / 1000.0
        self.running = False
        self.task: Optional[asyncio.Task] = None

        # Statistics
        self.total_detections_run = 0
        self.total_objects_detected = 0
        self.last_detection_time = 0

        logger.info(f"Detection loop initialized (interval: {interval_ms}ms)")

    def _decode_camera_frame(self, raw_data) -> Optional[np.ndarray]:
        """
        Decode raw camera data to numpy array for YOLO.

        Args:
            raw_data: Raw camera data (bytes or already decoded)

        Returns:
            Numpy array (BGR format for OpenCV/YOLO) or None if failed
        """
        try:
            if raw_data is None:
                return None

            # If it's already a numpy array, return it
            if isinstance(raw_data, np.ndarray):
                return raw_data

            # If it's bytes, decode it
            if isinstance(raw_data, bytes):
                # Convert bytes to numpy array
                nparr = np.frombuffer(raw_data, np.uint8)
                # Decode image
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                return frame

            logger.warning(f"Unknown camera data type: {type(raw_data)}")
            return None

        except Exception as e:
            logger.error(f"Error decoding camera frame: {e}")
            return None

    async def run(self):
        """Main detection loop - runs continuously."""
        self.running = True
        logger.info("Detection loop started")

        while self.running:
            try:
                # STEP 4.1: Pull latest fused RGB frame from state manager
                camera_frame = self.state_manager.last_camera_frame

                if camera_frame is None:
                    logger.debug("No camera frame available, skipping detection")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Check if camera data is fresh (within timeout)
                camera_timestamp = self.state_manager.last_camera_timestamp
                current_time = time.time()
                sensor_timeout = 5.0  # 5 seconds timeout

                if current_time - camera_timestamp > sensor_timeout:
                    logger.warning(f"Camera data is stale ({current_time - camera_timestamp:.1f}s old)")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # STEP 4.2: Decode frame for YOLO
                frame = self._decode_camera_frame(camera_frame)

                if frame is None:
                    logger.warning("Failed to decode camera frame")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                logger.debug(f"Running YOLO on frame shape: {frame.shape}")

                # STEP 4.3: Pass frame through YOLO model
                # Run in executor to avoid blocking the event loop
                loop = asyncio.get_event_loop()
                detections = await loop.run_in_executor(
                    None,
                    self.yolo_model.detect,
                    frame,
                    False  # verbose=False
                )

                self.total_detections_run += 1
                self.last_detection_time = current_time

                logger.info(f"YOLO detected {len(detections)} objects")

                # STEP 4.4: Filter detections by confidence threshold (>0.7)
                # Already done by YOLO model, but double-check
                filtered_detections = [
                    d for d in detections
                    if d.get('confidence', 0) >= self.yolo_model.confidence_threshold
                ]

                logger.debug(f"After filtering: {len(filtered_detections)} objects above threshold")

                # STEP 4.5: Enrich each detection with depth data from LiDAR
                # Get LiDAR data if available
                lidar_data = self.state_manager.last_lidar_data
                enriched_detections = []

                for detection in filtered_detections:
                    # Create DetectedObject (imported at top of file)
                    # Get depth from LiDAR if available
                    depth = None
                    if lidar_data:
                        # TODO: In Phase 3 (Sensor Fusion), we'll properly align
                        # LiDAR depth with camera pixels. For now, use placeholder.
                        # You can estimate depth from LiDAR point cloud center
                        depth = self._estimate_depth_from_lidar(detection, lidar_data)

                    obj = DetectedObject(
                        label=detection['label'],
                        confidence=detection['confidence'],
                        bbox=detection['bbox'],
                        depth=depth,
                        timestamp=current_time
                    )

                    enriched_detections.append(obj)

                # STEP 4.6: Store labeled objects in state manager
                # Clear old detections first (keep only last 10 seconds)
                self.state_manager.clear_old_detections(max_age_seconds=10.0)

                # Add new detections
                for obj in enriched_detections:
                    self.state_manager.add_detected_object(obj)
                    self.total_objects_detected += 1

                # STEP 4.7: Update environment snapshot for planning agent
                # (State manager automatically makes this available via get_detected_objects)

                logger.info(
                    f"Detection cycle complete: {len(enriched_detections)} objects stored. "
                    f"Total runs: {self.total_detections_run}, Total objects: {self.total_objects_detected}"
                )

            except Exception as e:
                logger.error(f"Error in detection loop: {e}", exc_info=True)

            # Wait for next cycle
            await asyncio.sleep(self.interval_seconds)

        logger.info("Detection loop stopped")

    def _estimate_depth_from_lidar(self, detection: dict, lidar_data: dict) -> Optional[float]:
        """
        Estimate depth for a detected object from LiDAR data.

        Uses a heuristic approach that filters LiDAR points by bounding box region.
        This assumes camera and LiDAR are roughly co-located (good enough for MVP).

        For proper sensor fusion, you would need:
        1. Camera intrinsics matrix (focal length, principal point)
        2. LiDAR-to-camera extrinsic calibration
        3. Project LiDAR 3D points to 2D camera pixels
        4. Filter points within bbox

        Args:
            detection: YOLO detection dict with bbox [x, y, width, height]
            lidar_data: LiDAR point cloud data with structure: {'points': [{'x': ..., 'y': ..., 'z': ...}]}

        Returns:
            Estimated depth in meters or None
        """
        try:
            if 'points' not in lidar_data or len(lidar_data['points']) == 0:
                return None

            # Extract bounding box (in pixel coordinates)
            bbox = detection.get('bbox', [])
            if len(bbox) != 4:
                return None

            x, y, width, height = bbox

            # Calculate bbox center in normalized coordinates (0-1)
            # Using camera resolution from config
            bbox_center_x = (x + width / 2) / settings.camera_width
            bbox_center_y = (y + height / 2) / settings.camera_height

            # Normalize bbox size
            bbox_width_norm = width / settings.camera_width
            bbox_height_norm = height / settings.camera_height

            # Filter LiDAR points by angular region that roughly corresponds to bbox
            # This is a simplified heuristic: we assume LiDAR FOV matches camera FOV
            # and filter points within an angular cone around bbox center

            relevant_depths = []

            for point in lidar_data['points']:
                # LiDAR points are in 3D space (x, y, z)
                # Assuming: x = horizontal, y = vertical, z = depth
                px = point.get('x', 0)
                py = point.get('y', 0)
                pz = point.get('z', 0)

                # Skip invalid points
                if pz <= 0 or pz > 10:  # Filter out points too close or too far (>10m)
                    continue

                # Convert 3D point to normalized angular coordinates
                # angle_x = arctan(x / z), normalized to 0-1
                # For simplicity, use linear approximation: x/z maps to pixel position

                # Calculate angular position (rough approximation)
                # Assuming FOV ~60 degrees horizontal, ~45 degrees vertical
                if pz > 0:
                    point_x_norm = 0.5 + (px / pz) * 0.5  # Map to 0-1
                    point_y_norm = 0.5 + (py / pz) * 0.5

                    # Check if point falls within bounding box region (with some margin)
                    margin = 0.1  # 10% margin for tolerance
                    x_min = bbox_center_x - (bbox_width_norm / 2) - margin
                    x_max = bbox_center_x + (bbox_width_norm / 2) + margin
                    y_min = bbox_center_y - (bbox_height_norm / 2) - margin
                    y_max = bbox_center_y + (bbox_height_norm / 2) + margin

                    if (x_min <= point_x_norm <= x_max and
                        y_min <= point_y_norm <= y_max):
                        relevant_depths.append(pz)

            # If we found points within the bbox, use median depth (robust to outliers)
            if relevant_depths:
                relevant_depths.sort()
                median_idx = len(relevant_depths) // 2
                median_depth = relevant_depths[median_idx]

                logger.debug(
                    f"Estimated depth for {detection.get('label')}: {median_depth:.2f}m "
                    f"from {len(relevant_depths)} LiDAR points"
                )
                return float(median_depth)

            # Fallback: if no points found in bbox region, use closest point to center
            # This is better than averaging ALL points (which was the old bug)
            min_distance = float('inf')
            closest_depth = None

            for point in lidar_data['points']:
                pz = point.get('z', 0)
                if pz <= 0 or pz > 10:
                    continue

                px = point.get('x', 0)
                py = point.get('y', 0)

                # Distance from bbox center (in angular space)
                if pz > 0:
                    point_x_norm = 0.5 + (px / pz) * 0.5
                    point_y_norm = 0.5 + (py / pz) * 0.5

                    dx = point_x_norm - bbox_center_x
                    dy = point_y_norm - bbox_center_y
                    distance = (dx**2 + dy**2) ** 0.5

                    if distance < min_distance:
                        min_distance = distance
                        closest_depth = pz

            if closest_depth:
                logger.debug(
                    f"Fallback: Using closest point depth {closest_depth:.2f}m for {detection.get('label')}"
                )
                return float(closest_depth)

            return None

        except Exception as e:
            logger.debug(f"Could not estimate depth: {e}")
            return None

    def start(self):
        """Start the detection loop as a background task."""
        if self.running:
            logger.warning("Detection loop already running")
            return

        self.task = asyncio.create_task(self.run())
        logger.info("Detection loop task created")

    async def stop(self):
        """Stop the detection loop."""
        self.running = False
        if self.task:
            await self.task
            logger.info("Detection loop stopped gracefully")

    def get_stats(self) -> dict:
        """Get detection loop statistics."""
        return {
            "total_runs": self.total_detections_run,
            "total_objects_detected": self.total_objects_detected,
            "last_detection_time": self.last_detection_time,
            "running": self.running
        }


# Standalone function for testing
async def test_detection_loop():
    """Test the detection loop with mock camera data."""
    from state_manager import state_manager
    from models.yolo_model import YOLOModel
    import cv2

    # Initialize YOLO
    logger.info("Loading YOLO model...")
    yolo = YOLOModel(model_path="yolo11n.pt", confidence_threshold=0.7)

    # Create mock camera frame (black image with white rectangle)
    mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(mock_frame, (200, 200), (400, 400), (255, 255, 255), -1)

    # Encode to bytes (simulating ESP32 camera)
    _, encoded = cv2.imencode('.jpg', mock_frame)
    mock_camera_data = encoded.tobytes()

    # Update state manager with mock data
    state_manager.update_camera(mock_camera_data, time.time())
    state_manager.update_lidar({'points': []}, time.time())

    # Create and run detection loop
    detection_loop = DetectionLoop(
        state_manager=state_manager,
        yolo_model=yolo,
        interval_ms=2000  # 2 seconds for testing
    )

    # Run for a few cycles
    detection_loop.start()
    await asyncio.sleep(6)  # Run 3 cycles
    await detection_loop.stop()

    # Check results
    stats = detection_loop.get_stats()
    print(f"\nDetection Loop Stats:")
    print(f"  Total runs: {stats['total_runs']}")
    print(f"  Total objects detected: {stats['total_objects_detected']}")

    detected_objects = state_manager.get_detected_objects()
    print(f"\nDetected objects in state manager: {len(detected_objects)}")
    for obj in detected_objects:
        print(f"  - {obj.label} (confidence: {obj.confidence:.2f}, depth: {obj.depth})")


if __name__ == "__main__":
    # Configure logging for test
    logging.basicConfig(level=logging.INFO)

    # Run test
    asyncio.run(test_detection_loop())
