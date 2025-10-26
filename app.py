from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import json
import base64
from datetime import datetime
from agents import LidarAnalysisAgent, DataProcessingAgent
from database import db, LidarScan
import config

app = Flask(__name__)
app.config.from_object(config.Config)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize database
db.init_app(app)

# Initialize agents
lidar_agent = LidarAnalysisAgent()
processing_agent = DataProcessingAgent()

# Store latest frame for streaming
latest_frame = None
frame_lock = None

try:
    import threading
    frame_lock = threading.Lock()
except:
    pass

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
        
        # Format analysis for iOS (convert to simple string)
        analysis_text = format_analysis_for_client(analysis)
        
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
            'analysis': analysis_text
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

def format_analysis_for_client(analysis):
    """Format analysis dictionary into a readable string for iOS client"""
    if isinstance(analysis, str):
        return analysis
    
    if not isinstance(analysis, dict):
        return str(analysis)
    
    parts = []
    
    # Extract raw analysis
    if 'raw_analysis' in analysis:
        parts.append(analysis['raw_analysis'])
    
    # Add confidence if available
    if 'confidence' in analysis:
        confidence_pct = analysis['confidence'] * 100
        parts.append(f"\n✓ Confidence: {confidence_pct:.1f}%")
    
    # Add structured analysis if available
    if 'structured' in analysis and isinstance(analysis['structured'], dict):
        structured = analysis['structured']
        if 'description' in structured:
            parts.append(f"\n📊 {structured['description']}")
        if 'quality' in structured:
            parts.append(f"Quality: {structured['quality']}")
        if 'recommendations' in structured and isinstance(structured['recommendations'], list):
            parts.append(f"\n💡 Recommendations:")
            for rec in structured['recommendations']:
                parts.append(f"  • {rec}")
    
    # Add fallback analysis if available
    if 'fallback_analysis' in analysis and isinstance(analysis['fallback_analysis'], dict):
        fallback = analysis['fallback_analysis']
        if 'description' in fallback:
            parts.append(fallback['description'])
        if 'quality' in fallback:
            parts.append(f"Quality: {fallback['quality']}")
    
    return '\n'.join(parts) if parts else 'Analysis completed successfully'


latest_frame = None
frame_lock = threading.Lock()

@app.route("/api/upload_frame", methods=["POST"])
def upload_frame():
    """Receive live camera frame from iOS device"""
    global latest_frame
    try:
        if not request.data:
            return jsonify({"error": "No image data"}), 400
        
        # Save binary JPEG frame
        with frame_lock:
            latest_frame = request.data

        return jsonify({"success": True}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/stream")
def stream_feed():
    """Stream MJPEG camera feed to dashboard"""
    def generate():
        global latest_frame
        while True:
            if latest_frame:
                with frame_lock:
                    frame = latest_frame
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            else:
                # Send blank placeholder
                import time
                time.sleep(0.05)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    with app.app_context():
        db.create_all()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)