from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import json
from datetime import datetime
from agents import LidarAnalysisAgent, DataProcessingAgent
from database import db, LidarScan
import config

app = Flask(__name__)
app.config.from_object(config.Config)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

db.init_app(app)

lidar_agent = LidarAnalysisAgent()
processing_agent = DataProcessingAgent()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/upload_lidar', methods=['POST'])
def upload_lidar():
    """Endpoint to receive LiDAR data from iOS app"""
    try:
        data = request.get_json()
        
        # Validate data
        if not data or 'points' not in data:
            return jsonify({'error': 'Invalid data format'}), 400
        
        # Process data with agent
        processed_data = processing_agent.process_lidar_data(data)
        
        # Save to database
        with app.app_context():
            scan = LidarScan(
                timestamp=datetime.utcnow(),
                point_count=len(data['points']),
                raw_data=json.dumps(data),
                processed_data=json.dumps(processed_data)
            )
            db.session.add(scan)
            db.session.commit()
            scan_id = scan.id
        
        # Analyze with AI agent
        analysis = lidar_agent.analyze_scan(processed_data)
        
        # Broadcast to connected clients
        socketio.emit('new_lidar_data', {
            'scan_id': scan_id,
            'data': processed_data,
            'analysis': analysis,
            'timestamp': datetime.utcnow().isoformat()
        })
        
        return jsonify({
            'success': True,
            'scan_id': scan_id,
            'analysis': analysis
        }), 200
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/scans', methods=['GET'])
def get_scans():
    """Get all LiDAR scans"""
    try:
        scans = LidarScan.query.order_by(LidarScan.timestamp.desc()).limit(50).all()
        return jsonify({
            'scans': [{
                'id': scan.id,
                'timestamp': scan.timestamp.isoformat(),
                'point_count': scan.point_count
            } for scan in scans]
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/scan/<int:scan_id>', methods=['GET'])
def get_scan(scan_id):
    """Get specific scan data"""
    try:
        scan = LidarScan.query.get_or_404(scan_id)
        return jsonify({
            'id': scan.id,
            'timestamp': scan.timestamp.isoformat(),
            'point_count': scan.point_count,
            'data': json.loads(scan.processed_data)
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/analyze/<int:scan_id>', methods=['POST'])
def analyze_scan(scan_id):
    """Trigger AI analysis on a specific scan"""
    try:
        scan = LidarScan.query.get_or_404(scan_id)
        processed_data = json.loads(scan.processed_data)
        
        analysis = lidar_agent.analyze_scan(processed_data)
        
        return jsonify({
            'scan_id': scan_id,
            'analysis': analysis
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('connection_response', {'status': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    with app.app_context():
        db.create_all()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)

