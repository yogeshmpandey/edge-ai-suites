"""
Streaming route handlers for the Live Video Captioning Dashboard.

Provides Server-Sent Events (SSE) endpoints for metadata and system stats.
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import Any

from aiohttp import web
from aiohttp.client_exceptions import ClientConnectionResetError
import psutil

from src.config import POLL_INTERVAL
from src.models.run import RunInfo
from src.services.metadata import read_latest_line
from src.services.gpu_collector import QmassaCollector, get_gpu_stats_dict

logger = logging.getLogger(__name__)


async def all_runs_metadata_stream(request: web.Request) -> web.StreamResponse:
    """
    ---
    summary: Stream metadata for all active runs (multiplexed SSE)
    description: |
      Server-Sent Events stream that combines metadata from all active runs
      into a single connection. Each event includes the run_id to identify
      which run the metadata belongs to. This reduces browser connection usage
      since browsers limit concurrent connections per host (typically 6).
    tags:
      - Runs
      - Streaming
    responses:
      "200":
        description: SSE stream of all runs metadata
        content:
          text/event-stream:
            schema:
              type: string
              description: |
                JSON object with runId and data fields.
                Example: {"runId": "abc123", "data": {...metadata...}}
    """
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": "text/event-stream",
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
        }
    )
    await response.prepare(request)
    
    # Track last payload per run to avoid duplicate sends
    last_payloads: dict[str, str] = {}
    
    try:
        while True:
            runs: dict[str, RunInfo] = request.app["runs"]
            current_runs = dict(runs)
            
            # Clean up tracking for removed runs
            removed_runs = set(last_payloads.keys()) - set(current_runs.keys())
            for run_id in removed_runs:
                last_payloads.pop(run_id, None)
                # Send a "removed" event so frontend knows to clean up
                event = {"runId": run_id, "removed": True}
                await response.write(f"data: {json.dumps(event)}\n\n".encode("utf-8"))
            
            # Send updates for all active runs
            for run_id, info in current_runs.items():
                try:
                    latest = await read_latest_line(Path(info.metadataFile))
                    if latest and latest != last_payloads.get(run_id):
                        last_payloads[run_id] = latest
                        # Parse and wrap with runId
                        try:
                            data = json.loads(latest)
                        except json.JSONDecodeError:
                            data = latest
                        event = {"runId": run_id, "data": data}
                        await response.write(f"data: {json.dumps(event)}\n\n".encode("utf-8"))
                except Exception as e:
                    # Skip this run if there's an error reading its metadata
                    logger.debug(f"Error reading metadata for run {run_id}: {e}")
            
            await asyncio.sleep(POLL_INTERVAL)
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        logger.debug("Metadata stream client disconnected")
    
    return response


async def system_stats(request: web.Request) -> web.StreamResponse:
    """
    ---
    summary: Stream system statistics (SSE)
    description: Server-Sent Events stream of CPU, memory, and GPU statistics.
    tags:
      - Monitoring
      - Streaming
    responses:
      "200":
        description: SSE stream of system stats
        content:
          text/event-stream:
            schema:
              type: string
    """
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": "text/event-stream",
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
        }
    )
    await response.prepare(request)
    
    try:
        while True:
            stats = await _collect_system_stats(request)
            await response.write(f"data: {json.dumps(stats)}\n\n".encode("utf-8"))
            await asyncio.sleep(POLL_INTERVAL)
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        logger.debug("System stats stream client disconnected")
    
    return response


async def _collect_system_stats(request: web.Request) -> dict[str, Any]:
    """Collect CPU, memory, and GPU statistics.
    
    Args:
        request: Current request (for accessing app context).
        
    Returns:
        Dictionary with system stats.
    """
    # Get CPU stats in thread to avoid blocking
    cpu = await asyncio.to_thread(psutil.cpu_percent, interval=0.5)
    mem = psutil.virtual_memory()

    # Get GPU stats from collector
    collector: QmassaCollector | None = request.app.get("gpu_collector")
    gpu_stats = get_gpu_stats_dict(collector)

    return {
        "cpu_percent": cpu,
        "mem_total_gb": round(mem.total / (1024**3), 2),
        "mem_used_gb": round(mem.used / (1024**3), 2),
        "mem_percent": mem.percent,
        # GPU metrics from qmassa
        "gpu_available": gpu_stats.get("available", False),
        "gpu_percent": gpu_stats.get("usage_percent"),
        "gpu_name": gpu_stats.get("device_name"),
        "gpu_driver": gpu_stats.get("driver"),
        "gpu_type": gpu_stats.get("device_type"),
        # GPU Engine breakdown
        "gpu_engines": gpu_stats.get("engines", {}),
        # VRAM
        "vram_used_gb": gpu_stats.get("vram_used_gb"),
        "vram_total_gb": gpu_stats.get("vram_total_gb"),
        "vram_percent": gpu_stats.get("vram_percent"),
        # System memory used by GPU
        "gpu_smem_used_gb": gpu_stats.get("smem_used_gb"),
        "gpu_smem_total_gb": gpu_stats.get("smem_total_gb"),
        # Frequency
        "gpu_freq_mhz": gpu_stats.get("freq_actual"),
        "gpu_freq_max_mhz": gpu_stats.get("freq_max"),
        # Power
        "gpu_power_w": gpu_stats.get("power_gpu"),
        "gpu_power_package_w": gpu_stats.get("power_package"),
        # Temperature
        "gpu_temp_c": gpu_stats.get("temperature"),
        # Error if any
        "gpu_error": gpu_stats.get("error"),
    }


# Route definitions for registration with Swagger
streaming_routes = [
    web.get("/api/runs/metadata-stream", all_runs_metadata_stream, allow_head=False),
    web.get("/api/system-stats", system_stats, allow_head=False),
]
