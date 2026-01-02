"""
Pipeline service for the Live Video Captioning Dashboard.

Provides functions for interacting with the DLStreamer Pipeline Server.
"""

import json
import logging
from typing import Any, Optional

import aiohttp

from src.config import PIPELINE_SERVER_URL, PIPELINE_NAME
from src.utils.http import HTTPException, http_json_async

logger = logging.getLogger(__name__)


async def start_pipeline(
    session: aiohttp.ClientSession,
    pipeline_name: str,
    source_uri: str,
    peer_id: str,
    metadata_file: str,
    prompt: str,
    model_name: str,
    max_new_tokens: int,
) -> str:
    """Start a video processing pipeline on the pipeline server.
    
    Args:
        session: aiohttp ClientSession to use.
        pipeline_name: Name of the pipeline to start.
        source_uri: RTSP URL of the video source.
        peer_id: WebRTC peer ID for video streaming.
        metadata_file: Path to write metadata output.
        prompt: Prompt for the captioning model.
        model_name: Name of the captioning model.
        max_new_tokens: Maximum tokens to generate.
        
    Returns:
        Pipeline instance ID from the server.
        
    Raises:
        HTTPException: On pipeline server errors.
    """
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/user_defined_pipelines/{pipeline_name}"
    
    payload = {
        "source": {"uri": source_uri, "type": "uri"},
        "destination": {
            "metadata": {"type": "file", "path": metadata_file, "format": "json-lines"},
            "frame": {"type": "webrtc", "peer-id": peer_id, "bitrate": 3000},
        },
        "parameters": {
            "captioner-prompt": prompt,
            "captioner_model_name": model_name,
            "captioner_max_new_tokens": max_new_tokens,
        },
    }
    
    logger.info(f"Starting pipeline '{pipeline_name}' at {url}")
    raw = await http_json_async(session, "POST", url, payload=payload, timeout=120)
    
    pipeline_id = raw.replace('"', "").strip()
    if not pipeline_id:
        logger.error(f"Pipeline server returned empty pipeline id: {raw}")
        raise HTTPException(
            status_code=502,
            detail={"message": "Pipeline server returned empty pipeline id", "body": raw}
        )
    
    logger.info(f"Pipeline started with id: {pipeline_id}")
    return pipeline_id


async def stop_pipeline(
    session: aiohttp.ClientSession,
    pipeline_id: str,
) -> bool:
    """Stop a video processing pipeline.
    
    Args:
        session: aiohttp ClientSession to use.
        pipeline_id: Pipeline instance ID to stop.
        
    Returns:
        True if stopped successfully, False if already stopped (404).
        
    Raises:
        HTTPException: On pipeline server errors (other than 404).
    """
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/{pipeline_id}"
    
    logger.info(f"Stopping pipeline {pipeline_id}")
    try:
        await http_json_async(session, "DELETE", url, timeout=15)
        logger.info(f"Pipeline {pipeline_id} stopped successfully")
        return True
    except HTTPException as e:
        # If pipeline server says 404, pipeline may already be stopped
        if e.status_code == 502 and e.detail and e.detail.get("status") == 404:
            logger.warning(f"Pipeline {pipeline_id} already stopped (404)")
            return False
        raise


async def get_pipeline_status(
    session: aiohttp.ClientSession,
    pipeline_id: str,
) -> Optional[dict[str, Any]]:
    """Get the status of a pipeline instance.
    
    Args:
        session: aiohttp ClientSession to use.
        pipeline_id: Pipeline instance ID to query.
        
    Returns:
        Pipeline status dict, or None if not found.
    """
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/{pipeline_id}/status"
    
    try:
        raw = await http_json_async(session, "GET", url, timeout=10)
        return json.loads(raw)
    except HTTPException as e:
        if e.status_code == 502 and e.detail and e.detail.get("status") == 404:
            return None
        raise
    except json.JSONDecodeError:
        return None


def validate_pipeline_response(response: Any) -> bool:
    """Validate that a pipeline server response has expected format.
    
    Args:
        response: Parsed JSON response from pipeline server.
        
    Returns:
        True if response format is valid.
    """
    if response is None:
        return False
    if isinstance(response, str):
        return len(response.strip()) > 0
    if isinstance(response, dict):
        # Check for common error indicators
        if "error" in response or "message" in response:
            return False
        return True
    return False
