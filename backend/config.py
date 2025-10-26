"""
Configuration module for loading environment variables and application settings.
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Gemini API Configuration
    gemini_api_key: str
    gemini_model: str = "gemini-2.0-flash-exp"

    # YOLO Configuration
    yolo_model_path: str = "yolov8n.pt"
    confidence_threshold: float = 0.7

    # Sensor Configuration
    sensor_timeout_ms: int = 2000
    lidar_ws_port: int = 8001
    camera_ws_port: int = 8002
    camera_width: int = 640
    camera_height: int = 480

    # Server Configuration
    server_host: str = "0.0.0.0"
    server_port: int = 8000
    debug: bool = True

    # Loop Intervals (milliseconds)
    sensor_loop_interval: int = 500
    fusion_loop_interval: int = 500
    detection_loop_interval: int = 3000
    planning_loop_interval: int = 1000
    output_loop_interval: int = 3000

    # Mission Configuration
    default_objective: str = "Find a red coffee mug"
    success_distance_threshold: float = 0.5
    success_confidence_threshold: float = 0.95

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"  # Ignore extra fields in .env


# Global settings instance
settings = Settings()
