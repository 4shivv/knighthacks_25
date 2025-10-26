from flask import Flask, render_template, request, jsonify, Response
from flask_cors import CORS
from flask_socketio import SocketIO
import json, time, threading, io
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from agents import LidarAnalysisAgent, DataProcessingAgent
from database import db, LidarScan
import config
import matplotlib
matplotlib.use("Agg")

app = Flask(__name__)
app.config.from_object(config.Config)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize DB + agents
db.init_app(app)
lidar_agent = LidarAnalysisAgent()
processing_agent = DataProcessingAgent()

# Globals
latest_frame = None
latest_lidar_points = None
frame_lock = threading.Lock()

@app.route('/')
def index():
    return render_template('index.html')

# ---------------- CAMERA STREAM ----------------
@app.route("/api/upload_frame", methods=["POST"])
def upload_frame():
    """Receive live camera frame from iOS device"""
    global latest_frame
    try:
        if not request.data:
            return jsonify({"error": "No image data"}), 400
        with frame_lock:
            latest_frame = request.data
        return jsonify({"success": True}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/stream_camera")
def stream_camera():
    """Stream MJPEG camera feed"""
    def generate():
        global latest_frame
        while True:
            if latest_frame:
                with frame_lock:
                    frame = latest_frame
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            else:
                time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- LIDAR STREAM ----------------
@app.route("/api/upload_lidar_frame", methods=["POST"])
def upload_lidar_frame():
    """Receive LiDAR JSON points from iOS app"""
    global latest_lidar_points
    try:
        data = request.get_json()
        if not data or "points" not in data:
            return jsonify({"error": "Invalid JSON"}), 400
        pts = np.array([[p["x"], p["y"], p["z"]] for p in data["points"]])
        latest_lidar_points = pts
        return jsonify({"success": True, "count": len(pts)}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/stream_lidar")
def stream_lidar():
    """Render LiDAR points as scatter MJPEG stream"""
    def generate():
        global latest_lidar_points
        while True:
            if latest_lidar_points is not None and len(latest_lidar_points) > 0:
                pts = latest_lidar_points

                # Create scatter plot
                fig, ax = plt.subplots(figsize=(5, 5))
                ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], cmap='plasma', s=4)
                ax.set_xlim(np.min(pts[:, 0]) - 0.5, np.max(pts[:, 0]) + 0.5)
                ax.set_ylim(np.min(pts[:, 1]) - 0.5, np.max(pts[:, 1]) + 0.5)
                ax.axis('off')
                plt.tight_layout(pad=0)

                # Convert to JPEG
                buf = io.BytesIO()
                fig.savefig(buf, format='jpeg', dpi=80)
                plt.close(fig)
                frame = buf.getvalue()
                buf.close()

                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            else:
                time.sleep(0.1)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- SCAN DATA ----------------
@app.route('/api/scans', methods=['GET'])
def get_scans():
    try:
        scans = LidarScan.query.order_by(LidarScan.timestamp.desc()).limit(50).all()
        return jsonify({
            'scans': [{
                'id': s.id,
                'timestamp': s.timestamp.isoformat(),
                'point_count': s.point_count
            } for s in scans]
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    with app.app_context():
        db.create_all()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
