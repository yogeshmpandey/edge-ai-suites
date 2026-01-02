"""
Live Video Captioning Dashboard

This package provides an async HTTP server using aiohttp for the live video
captioning dashboard. It exposes REST APIs for managing video processing
pipelines, streaming metadata, and monitoring system/GPU metrics.

Module Import Rules:
    - models/ imports only from standard library and dataclasses
    - utils/ imports only from standard library and aiohttp
    - services/ imports from models/, utils/, and standard library
    - routes/ imports from services/, models/, utils/, and aiohttp
    - app.py imports from routes/, services/, and handles lifecycle

This ensures no circular imports and clean dependency flow:
    models → utils → services → routes → app
"""

__version__ = "1.0.0"
