"""
Environment configuration and constants for the Live Video Captioning Dashboard.

All configuration is loaded from environment variables with sensible defaults.
"""

import os
from pathlib import Path

# Server configuration
APP_PORT: int = int(os.environ.get("DASHBOARD_PORT", "4173"))

# File paths
BASE_DIR: Path = Path(__file__).parent
MODELS_DIR: Path = Path(os.environ.get("MODELS_DIR", str(BASE_DIR / "ov_models")))
PUBLIC_DIR: Path = BASE_DIR / "public"

# Metadata configuration
METADATA_FILE: str = os.environ.get("METADATA_FILE", "/tmp/results.jsonl")
POLL_INTERVAL: float = float(os.environ.get("METADATA_POLL_SECONDS", "1"))

# WebRTC configuration
PEER_ID: str = os.environ.get("WEBRTC_PEER_ID", "genai_pipeline")
SIGNALING_URL: str = os.environ.get("SIGNALING_URL", "http://localhost:8889")

# Pipeline server configuration
PIPELINE_SERVER_URL: str = os.environ.get("PIPELINE_SERVER_URL", "http://video-ingestion:8080")
PIPELINE_NAME: str = os.environ.get("PIPELINE_NAME", "genai_pipeline")

# GPU metrics configuration
# Qmassa runs on the host with sudo to see all GPU clients across containers
# The dashboard reads from the host-generated JSON file mounted at /tmp
QMASSA_ENABLED: bool = os.environ.get("QMASSA_ENABLED", "true").lower() in ("true", "1", "yes")
QMASSA_JSON_FILE: Path = Path(os.environ.get("QMASSA_JSON_FILE", "/tmp/qmassa-stats.json"))
QMASSA_POLL_INTERVAL_MS: int = int(os.environ.get("QMASSA_POLL_INTERVAL_MS", "1000"))
