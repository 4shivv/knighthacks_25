"""
Vision processing with OpenCV for camera data.
PEP 8 compliant.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class VisionProcessor(Node):
    """Process camera images for object detection."""

    def __init__(self):
        """Initialize vision processor node."""
        super().__init__('vision_processor')
        
        # Parameters
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('image_processing_rate', 10.0)
        
        self.detection_threshold = self.get_parameter(
            'detection_threshold').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String, 'vision_detections', 10)
        self.debug_image_pub = self.create_publisher(
            Image, 'vision_debug', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # Detection state
        self.last_detections = []
        
        self.get_logger().info('Vision processor initialized')

    def image_callback(self, msg: Image) -> None:
        """Process camera image.
        
        Args:
            msg: Image message
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Process image for obstacle detection
        detections = self._detect_obstacles(cv_image)
        
        # Publish detections
        if detections:
            self.last_detections = detections
            detection_msg = String()
            detection_msg.data = json.dumps({
                'timestamp': msg.header.stamp.sec +
                            msg.header.stamp.nanosec * 1e-9,
                'detections': detections
            })
            self.detection_pub.publish(detection_msg)
        
        # Publish debug image
        debug_image = self._draw_detections(cv_image, detections)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image error: {e}')

    def _detect_obstacles(self, image: np.ndarray) -> list:
        """Detect obstacles in image using color segmentation.
        
        Args:
            image: BGR image from camera
            
        Returns:
            List of detection dictionaries
        """
        detections = []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for obstacles (red and blue)
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([130, 255, 255])
        
        # Create masks
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # Process each mask
        for mask, color_name in [(red_mask, 'red'), (blue_mask, 'blue')]:
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Estimate distance based on size
                    distance = self._estimate_distance(w, h)
                    
                    # Calculate angle from center
                    image_center_x = image.shape[1] // 2
                    angle = (center_x - image_center_x) / image.shape[1] * 60
                    
                    detections.append({
                        'color': color_name,
                        'bbox': [int(x), int(y), int(w), int(h)],
                        'center': [int(center_x), int(center_y)],
                        'distance': float(distance),
                        'angle': float(angle),
                        'area': float(area)
                    })
        
        return detections

    def _estimate_distance(self, width: int, height: int) -> float:
        """Estimate distance to object based on size.
        
        Args:
            width: Bounding box width
            height: Bounding box height
            
        Returns:
            Estimated distance in meters
        """
        # Simple distance estimation (calibration needed for real setup)
        avg_size = (width + height) / 2
        if avg_size > 0:
            distance = 1000.0 / avg_size  # Rough approximation
            return max(0.5, min(distance, 10.0))
        return 5.0

    def _draw_detections(self, image: np.ndarray,
                        detections: list) -> np.ndarray:
        """Draw detection bounding boxes on image.
        
        Args:
            image: Original image
            detections: List of detections
            
        Returns:
            Image with drawn detections
        """
        debug_image = image.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            color = (0, 0, 255) if det['color'] == 'red' else (255, 0, 0)
            
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            
            label = (f"{det['color']} "
                    f"d:{det['distance']:.1f}m "
                    f"a:{det['angle']:.0f}°")
            cv2.putText(debug_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return debug_image


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = VisionProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
