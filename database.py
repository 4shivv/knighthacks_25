from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

db = SQLAlchemy()


class LidarScan(db.Model):
    """Model for storing LiDAR scan data"""
    
    __tablename__ = 'lidar_scans'
    
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow, index=True)
    point_count = db.Column(db.Integer, nullable=False)
    raw_data = db.Column(db.Text, nullable=False)  # JSON string
    processed_data = db.Column(db.Text, nullable=True)  # JSON string
    device_info = db.Column(db.String(255), nullable=True)
    
    # Relationships
    analyses = db.relationship('ScanAnalysis', backref='scan', lazy=True, cascade='all, delete-orphan')
    
    def __repr__(self):
        return f'<LidarScan {self.id} - {self.timestamp} - {self.point_count} points>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'timestamp': self.timestamp.isoformat(),
            'point_count': self.point_count,
            'device_info': self.device_info
        }


class ScanAnalysis(db.Model):
    """Model for storing AI analysis results"""
    
    __tablename__ = 'scan_analyses'
    
    id = db.Column(db.Integer, primary_key=True)
    scan_id = db.Column(db.Integer, db.ForeignKey('lidar_scans.id'), nullable=False, index=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    analysis_type = db.Column(db.String(50), nullable=False)  # e.g., 'scene_description', 'quality_check'
    result = db.Column(db.Text, nullable=False)  # JSON string
    confidence = db.Column(db.Float, nullable=True)
    
    def __repr__(self):
        return f'<ScanAnalysis {self.id} - Scan {self.scan_id} - {self.analysis_type}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'scan_id': self.scan_id,
            'timestamp': self.timestamp.isoformat(),
            'analysis_type': self.analysis_type,
            'confidence': self.confidence
        }


class DeviceSession(db.Model):
    """Model for tracking iOS device sessions"""
    
    __tablename__ = 'device_sessions'
    
    id = db.Column(db.Integer, primary_key=True)
    device_id = db.Column(db.String(255), nullable=False, unique=True, index=True)
    device_name = db.Column(db.String(255), nullable=True)
    first_connection = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    last_connection = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    total_scans = db.Column(db.Integer, default=0)
    
    def __repr__(self):
        return f'<DeviceSession {self.device_id} - {self.device_name}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'device_id': self.device_id,
            'device_name': self.device_name,
            'first_connection': self.first_connection.isoformat(),
            'last_connection': self.last_connection.isoformat(),
            'total_scans': self.total_scans
        }