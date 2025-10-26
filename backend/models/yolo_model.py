"""
YOLO model wrapper for object detection.
Provides a clean interface for loading and running YOLO inference.
"""

from ultralytics import YOLO
from typing import List, Dict, Any, Optional
import numpy as np
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


class YOLOModel:
    """
    Wrapper for YOLO object detection model.
    Handles model loading, inference, and result processing.
    """

    def __init__(self, model_path: str = "yolo11n.pt", confidence_threshold: float = 0.7):
        """
        Initialize YOLO model.

        Args:
            model_path: Path to YOLO11 model weights (default: yolo11n.pt - fastest)
                       Options: yolo11n.pt (nano, fastest, 1.5ms)
                               yolo11s.pt (small, 2.5ms)
                               yolo11m.pt (medium, 4.7ms)
                               yolo11l.pt (large, 6.2ms)
                               yolo11x.pt (xlarge, 11.3ms)
            confidence_threshold: Minimum confidence for detections (0.0-1.0)
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model: Optional[YOLO] = None
        self._load_model()

    def _load_model(self):
        """Load YOLO model into memory."""
        try:
            logger.info(f"Loading YOLO model from {self.model_path}...")
            self.model = YOLO(self.model_path)
            logger.info("YOLO model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise

    def detect(self, frame: np.ndarray, verbose: bool = False) -> List[Dict[str, Any]]:
        """
        Run object detection on a frame.

        Args:
            frame: Input image as numpy array (RGB)
            verbose: Whether to print detection details

        Returns:
            List of detected objects with format:
            [
                {
                    'label': 'mug',
                    'confidence': 0.92,
                    'bbox': [x, y, width, height],
                    'center': [cx, cy]
                },
                ...
            ]
        """
        if self.model is None:
            logger.error("YOLO model not loaded")
            return []

        try:
            # Run inference
            results = self.model(frame, verbose=verbose)

            # Parse results
            detections = []
            for result in results:
                boxes = result.boxes

                for box in boxes:
                    confidence = float(box.conf[0])

                    # Filter by confidence threshold
                    if confidence < self.confidence_threshold:
                        continue

                    # Get bounding box coordinates
                    xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                    x1, y1, x2, y2 = xyxy

                    # Convert to [x, y, width, height]
                    bbox = [float(x1), float(y1), float(x2 - x1), float(y2 - y1)]

                    # Calculate center point
                    center = [float((x1 + x2) / 2), float((y1 + y2) / 2)]

                    # Get class label
                    class_id = int(box.cls[0])
                    label = result.names[class_id]

                    detection = {
                        'label': label,
                        'confidence': confidence,
                        'bbox': bbox,
                        'center': center,
                        'class_id': class_id
                    }

                    detections.append(detection)

            if verbose and detections:
                logger.info(f"Detected {len(detections)} objects")

            return detections

        except Exception as e:
            logger.error(f"Error during YOLO detection: {e}")
            return []

    def detect_specific_object(
        self,
        frame: np.ndarray,
        target_label: str,
        min_confidence: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Detect specific object by label.

        Args:
            frame: Input image
            target_label: Label to search for (e.g., 'cup', 'person')
            min_confidence: Override default confidence threshold

        Returns:
            List of detections matching the target label
        """
        all_detections = self.detect(frame)

        # Filter by label
        target_detections = [
            d for d in all_detections
            if d['label'].lower() == target_label.lower()
        ]

        # Apply custom confidence threshold if provided
        if min_confidence is not None:
            target_detections = [
                d for d in target_detections
                if d['confidence'] >= min_confidence
            ]

        return target_detections

    def get_model_info(self) -> Dict[str, Any]:
        """Get information about the loaded model."""
        if self.model is None:
            return {"loaded": False}

        return {
            "loaded": True,
            "model_path": self.model_path,
            "confidence_threshold": self.confidence_threshold,
            "model_type": str(type(self.model).__name__)
        }

    def update_confidence_threshold(self, new_threshold: float):
        """Update confidence threshold for detections."""
        if 0.0 <= new_threshold <= 1.0:
            self.confidence_threshold = new_threshold
            logger.info(f"Updated confidence threshold to {new_threshold}")
        else:
            logger.warning(f"Invalid threshold {new_threshold}, must be between 0 and 1")

    def reload_model(self):
        """Reload the YOLO model (useful for error recovery)."""
        logger.info("Reloading YOLO model...")
        self._load_model()
