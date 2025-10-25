import google.generativeai as genai
from typing import Dict, List, Any
import numpy as np
import config

# Configure Google AI
genai.configure(api_key=config.GOOGLE_API_KEY)

class LidarAnalysisAgent:
    """Agent for analyzing LiDAR scan data using Google ADK"""
    
    def __init__(self):
        self.model = genai.GenerativeModel('gemini-pro')
        self.chat = self.model.start_chat(history=[])
        
    def analyze_scan(self, processed_data: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze processed LiDAR data and provide insights"""
        
        # Prepare analysis prompt
        prompt = f"""
        You are a LiDAR data analysis expert. Analyze the following 3D point cloud data:
        
        Statistics:
        - Total Points: {processed_data.get('point_count', 0)}
        - Bounding Box: {processed_data.get('bounding_box', {})}
        - Density: {processed_data.get('density', 'N/A')}
        
        Point Distribution:
        - X Range: {processed_data.get('ranges', {}).get('x', [])}
        - Y Range: {processed_data.get('ranges', {}).get('y', [])}
        - Z Range: {processed_data.get('ranges', {}).get('z', [])}
        
        Provide:
        1. Scene description (what objects/surfaces are likely present)
        2. Data quality assessment
        3. Recommended processing steps
        4. Potential issues or anomalies
        
        Format your response as JSON with keys: description, quality, recommendations, issues
        """
        
        try:
            response = self.chat.send_message(prompt)
            
            # Parse response
            analysis = {
                'raw_analysis': response.text,
                'confidence': 0.85,
                'timestamp': processed_data.get('timestamp')
            }
            
            # Extract structured insights
            if 'description' in response.text.lower():
                analysis['structured'] = self._parse_structured_response(response.text)
            
            return analysis
            
        except Exception as e:
            return {
                'error': str(e),
                'fallback_analysis': self._basic_analysis(processed_data)
            }
    
    def _parse_structured_response(self, text: str) -> Dict:
        """Parse structured response from AI"""
        # Simple parsing logic - enhance as needed
        return {
            'parsed': True,
            'content': text
        }
    
    def _basic_analysis(self, data: Dict) -> Dict:
        """Fallback analysis without AI"""
        point_count = data.get('point_count', 0)
        
        quality = "good" if point_count > 1000 else "low"
        
        return {
            'description': f'Point cloud with {point_count} points',
            'quality': quality,
            'recommendations': ['Collect more data' if point_count < 1000 else 'Data sufficient'],
            'issues': []
        }


class DataProcessingAgent:
    """Agent for processing raw LiDAR data"""
    
    def __init__(self):
        self.model = genai.GenerativeModel('gemini-pro')
        
    def process_lidar_data(self, raw_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process raw LiDAR data from iOS"""
        
        points = raw_data.get('points', [])
        
        if not points:
            return {'error': 'No points in data'}
        
        # Convert to numpy for processing
        points_array = np.array(points)
        
        # Calculate statistics
        processed = {
            'point_count': len(points),
            'timestamp': raw_data.get('timestamp'),
            'device_info': raw_data.get('device_info', {}),
            'ranges': {
                'x': [float(points_array[:, 0].min()), float(points_array[:, 0].max())],
                'y': [float(points_array[:, 1].min()), float(points_array[:, 1].max())],
                'z': [float(points_array[:, 2].min()), float(points_array[:, 2].max())]
            },
            'centroid': [
                float(points_array[:, 0].mean()),
                float(points_array[:, 1].mean()),
                float(points_array[:, 2].mean())
            ],
            'bounding_box': self._calculate_bounding_box(points_array),
            'density': self._calculate_density(points_array)
        }
        
        # Sample points for visualization (limit to 10000 points)
        if len(points) > 10000:
            indices = np.random.choice(len(points), 10000, replace=False)
            processed['sampled_points'] = points_array[indices].tolist()
        else:
            processed['sampled_points'] = points
        
        return processed
    
    def _calculate_bounding_box(self, points: np.ndarray) -> Dict:
        """Calculate 3D bounding box"""
        return {
            'min': [float(points[:, 0].min()), float(points[:, 1].min()), float(points[:, 2].min())],
            'max': [float(points[:, 0].max()), float(points[:, 1].max()), float(points[:, 2].max())],
            'dimensions': [
                float(points[:, 0].max() - points[:, 0].min()),
                float(points[:, 1].max() - points[:, 1].min()),
                float(points[:, 2].max() - points[:, 2].min())
            ]
        }
    
    def _calculate_density(self, points: np.ndarray) -> float:
        """Calculate point cloud density"""
        if len(points) < 2:
            return 0.0
        
        # Calculate volume of bounding box
        ranges = [
            points[:, 0].max() - points[:, 0].min(),
            points[:, 1].max() - points[:, 1].min(),
            points[:, 2].max() - points[:, 2].min()
        ]
        
        volume = np.prod(ranges)
        
        if volume == 0:
            return 0.0
        
        return float(len(points) / volume)


class ScanComparisonAgent:
    """Agent for comparing multiple LiDAR scans"""
    
    def __init__(self):
        self.model = genai.GenerativeModel('gemini-pro')
        
    def compare_scans(self, scan1: Dict, scan2: Dict) -> Dict[str, Any]:
        """Compare two LiDAR scans and identify changes"""
        
        prompt = f"""
        Compare these two LiDAR scans and identify significant differences:
        
        Scan 1:
        - Points: {scan1.get('point_count')}
        - Bounding Box: {scan1.get('bounding_box')}
        
        Scan 2:
        - Points: {scan2.get('point_count')}
        - Bounding Box: {scan2.get('bounding_box')}
        
        Identify:
        1. Major geometric changes
        2. New or removed objects
        3. Movement or displacement
        """
        
        try:
            response = self.model.generate_content(prompt)
            
            return {
                'comparison': response.text,
                'point_difference': scan2.get('point_count', 0) - scan1.get('point_count', 0),
                'spatial_change': self._calculate_spatial_change(scan1, scan2)
            }
        except Exception as e:
            return {'error': str(e)}
    
    def _calculate_spatial_change(self, scan1: Dict, scan2: Dict) -> float:
        """Calculate spatial difference between scans"""
        # Simplified calculation
        return 0.0