"""
Shared utilities for the Live Video Captioning Dashboard.

This module contains:
    - http: HTTPException, async HTTP client, error response helper
"""

from src.utils.http import HTTPException, error_response

__all__ = ["HTTPException", "error_response"]
