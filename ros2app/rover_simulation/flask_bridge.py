"""
Flask bridge for external command interface.
PEP 8 compliant.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading
import json
from typing import Dict, Any


class FlaskBridge(Node):
    """Bridge between Flask app and ROS2 nodes."""

    def __init__(self):
        """Initialize Flask bridge node."""
        super().__init__('flask_bridge')
        
        # Parameters
        self.declare_parameter('flask_host', '0.0.0.0')
        self.declare_parameter('flask_port', 5000)
        
        self.flask_host = self.get_parameter('flask_host').value
        self.flask_port = self.get_parameter('flask_port').value
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String, 'rover_status', self.status_callback, 10)
        self.vision_sub = self.create_subscription(
            String, 'vision_detections', self.vision_callback, 10)
        
        # Shared state
        self.latest_status: Dict[str, Any] = {}
        self.latest_detections: Dict[str, Any] = {}
        self.command_queue: list = []
        
        # Start Flask in separate thread
        self.flask_thread = threading.Thread(
            target=self._run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info(
            f'Flask bridge initialized on {self.flask_host}:'
            f'{self.flask_port}')

    def status_callback(self, msg: String) -> None:
        """Update rover status.
        
        Args:
            msg: Status message from rover controller
        """
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid status JSON')

    def vision_callback(self, msg: String) -> None:
        """Update vision detections.
        
        Args:
            msg: Detection message from vision processor
        """
        try:
            self.latest_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid detection JSON')

    def set_goal(self, x: float, y: float, z: float = 0.0) -> bool:
        """Set new goal position.
        
        Args:
            x: Goal x coordinate
            y: Goal y coordinate
            z: Goal z coordinate (default 0.0)
            
        Returns:
            True if goal was published successfully
        """
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(msg)
            self.get_logger().info(f'Goal published: ({x}, {y}, {z})')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to publish goal: {e}')
            return False

    def get_status(self) -> Dict[str, Any]:
        """Get current rover status.
        
        Returns:
            Dictionary with rover status
        """
        return {
            'rover_status': self.latest_status,
            'vision_detections': self.latest_detections
        }

    def _run_flask(self) -> None:
        """Run Flask server (to be called in separate thread)."""
        # Import Flask here to avoid issues if not installed
        try:
            from flask import Flask, request, jsonify
            from flask_cors import CORS
        except ImportError:
            self.get_logger().error(
                'Flask not installed. Install with: pip install flask '
                'flask-cors')
            return
        
        app = Flask(__name__)
        CORS(app)
        
        @app.route('/status', methods=['GET'])
        def get_status():
            """Get rover status endpoint."""
            return jsonify(self.get_status())
        
        @app.route('/goal', methods=['POST'])
        def set_goal():
            """Set goal position endpoint."""
            data = request.json
            if not data or 'x' not in data or 'y' not in data:
                return jsonify({'error': 'Missing x or y coordinates'}), 400
            
            x = data['x']
            y = data['y']
            z = data.get('z', 0.0)
            
            success = self.set_goal(x, y, z)
            
            if success:
                return jsonify({
                    'success': True,
                    'goal': {'x': x, 'y': y, 'z': z}
                })
            else:
                return jsonify({'error': 'Failed to set goal'}), 500
        
        @app.route('/commands', methods=['POST'])
        def execute_command():
            """Execute custom command endpoint."""
            data = request.json
            if not data or 'command' not in data:
                return jsonify({'error': 'Missing command'}), 400
            
            command = data['command']
            params = data.get('params', {})
            
            # Process commands
            if command == 'stop':
                self.set_goal(0.0, 0.0, 0.0)  # Dummy goal at origin
                return jsonify({'success': True, 'message': 'Stop command'})
            elif command == 'move_to':
                if 'x' in params and 'y' in params:
                    success = self.set_goal(
                        params['x'], params['y'], params.get('z', 0.0))
                    return jsonify({'success': success})
                else:
                    return jsonify({'error': 'Missing coordinates'}), 400
            else:
                return jsonify({'error': 'Unknown command'}), 400
        
        @app.route('/health', methods=['GET'])
        def health_check():
            """Health check endpoint."""
            return jsonify({'status': 'healthy', 'node': 'flask_bridge'})
        
        try:
            app.run(host=self.flask_host, port=self.flask_port,
                   debug=False, use_reloader=False)
        except Exception as e:
            self.get_logger().error(f'Flask server error: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = FlaskBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()