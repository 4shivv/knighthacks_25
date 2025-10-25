#!/usr/bin/env python3
"""
Standalone Flask server for testing (optional).
PEP 8 compliant.
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import requests
import json


app = Flask(__name__)
CORS(app)

# Configuration
ROS_BRIDGE_URL = 'http://localhost:5000'


@app.route('/')
def index():
    """Root endpoint."""
    return jsonify({
        'service': 'Rover Simulation Flask Interface',
        'version': '1.0.0',
        'endpoints': [
            '/status',
            '/goal',
            '/commands',
            '/health'
        ]
    })


@app.route('/api/lidar', methods=['POST'])
def receive_lidar():
    """Receive LiDAR data from external source."""
    data = request.json
    if not data:
        return jsonify({'error': 'No data received'}), 400
    
    # Forward to ROS bridge or process
    print(f"Received LiDAR data: {len(data.get('points', []))} points")
    
    return jsonify({'success': True, 'processed': True})


@app.route('/api/camera', methods=['POST'])
def receive_camera():
    """Receive camera data from external source."""
    data = request.json
    if not data:
        return jsonify({'error': 'No data received'}), 400
    
    # Forward to ROS bridge or process
    print(f"Received camera data: {data.get('timestamp', 'unknown')}")
    
    return jsonify({'success': True, 'processed': True})


@app.route('/api/command', methods=['POST'])
def send_command():
    """Send command to rover."""
    data = request.json
    if not data or 'command' not in data:
        return jsonify({'error': 'Missing command'}), 400
    
    command = data['command']
    params = data.get('params', {})
    
    # Forward command to ROS bridge
    try:
        if command == 'move_to' and 'x' in params and 'y' in params:
            response = requests.post(
                f'{ROS_BRIDGE_URL}/goal',
                json={'x': params['x'], 'y': params['y']},
                timeout=5
            )
            return jsonify(response.json())
        else:
            response = requests.post(
                f'{ROS_BRIDGE_URL}/commands',
                json={'command': command, 'params': params},
                timeout=5
            )
            return jsonify(response.json())
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'ROS bridge error: {str(e)}'}), 500


@app.route('/api/status', methods=['GET'])
def get_rover_status():
    """Get rover status from ROS bridge."""
    try:
        response = requests.get(f'{ROS_BRIDGE_URL}/status', timeout=5)
        return jsonify(response.json())
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'ROS bridge error: {str(e)}'}), 500


if __name__ == '__main__':
    print("Starting Flask server on http://0.0.0.0:8000")
    print(f"ROS Bridge URL: {ROS_BRIDGE_URL}")
    app.run(host='0.0.0.0', port=8000, debug=True)