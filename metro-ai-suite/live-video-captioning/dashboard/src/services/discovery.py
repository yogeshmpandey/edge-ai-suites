"""
Discovery service for the Live Video Captioning Dashboard.

Provides functions to discover available models and pipelines.
"""

import json
import logging
from pathlib import Path
from typing import Any

import aiohttp

from src.config import PIPELINE_SERVER_URL, PIPELINE_NAME
from src.utils.http import http_json_async

logger = logging.getLogger(__name__)


def discover_models(root: Path) -> list[str]:
    """Discover available models from the models directory.
    
    Scans the given directory for model subdirectories and files.
    Ignores hidden files/directories (starting with '.').
    
    Args:
        root: Path to the models directory.
        
    Returns:
        List of model names found in the directory.
    """
    if not root.exists():
        return []
    
    models: list[str] = []
    for entry in sorted(root.iterdir()):
        if entry.name.startswith("."):
            continue
        if entry.is_dir():
            models.append(entry.name)
        else:
            # Allow flat exports placed directly under ov_models
            if entry.suffix in {".xml", ".bin", ".json"}:
                models.append(entry.name)
    return models


async def discover_pipelines_remote(session: aiohttp.ClientSession) -> list[str]:
    """Discover available pipelines from the pipeline server.
    
    Queries the pipeline server for available pipeline definitions.
    Falls back to the default pipeline name on error.
    
    Args:
        session: aiohttp ClientSession to use for the request.
        
    Returns:
        List of pipeline names, or [PIPELINE_NAME] on error.
    """
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines"
    try:
        raw = await http_json_async(session, "GET", url, timeout=10)
        payload = json.loads(raw)
        return _parse_pipelines_response(payload)
    except Exception as e:
        logger.warning(f"Failed to discover pipelines: {e}")
        return [PIPELINE_NAME]


def _parse_pipelines_response(payload: Any) -> list[str]:
    """Parse the pipeline server response to extract pipeline names.
    
    Handles various response formats from the pipeline server.
    
    Args:
        payload: Parsed JSON response from pipeline server.
        
    Returns:
        List of pipeline names.
    """
    # Accept either list[str] or list[dict {version}] or {'pipelines': [...]}
    if isinstance(payload, list):
        names = []
        for item in payload:
            if isinstance(item, str):
                names.append(item)
            elif isinstance(item, dict) and isinstance(item.get("version"), str):
                names.append(item["version"])
        return names or [PIPELINE_NAME]
    
    if isinstance(payload, dict):
        items = payload.get("pipelines") or payload.get("items") or []
        if isinstance(items, list):
            names = []
            for item in items:
                if isinstance(item, str):
                    names.append(item)
                elif isinstance(item, dict) and isinstance(item.get("version"), str):
                    names.append(item["version"])
            return names or [PIPELINE_NAME]
    
    return [PIPELINE_NAME]
