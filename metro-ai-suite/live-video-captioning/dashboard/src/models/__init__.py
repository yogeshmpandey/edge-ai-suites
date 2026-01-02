"""
Data models and entities for the Live Video Captioning Dashboard.

This module contains dataclasses for:
    - StartRunRequest: Request payload for creating runs
    - RunInfo: Active run state
    - GPUMetrics: GPU telemetry data
"""

from src.models.run import RunInfo, StartRunRequest
from src.models.gpu import GPUMetrics

__all__ = ["RunInfo", "StartRunRequest", "GPUMetrics"]
