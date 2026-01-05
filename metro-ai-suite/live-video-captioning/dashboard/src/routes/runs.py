"""
Runs route handlers for the Live Video Captioning Dashboard.

Provides CRUD endpoints for managing video processing runs.
"""

import json
import logging
import uuid
from pathlib import Path
from typing import Any

from aiohttp import web

from src.config import PIPELINE_NAME
from src.models.run import RunInfo, StartRunRequest
from src.services.pipeline import start_pipeline, stop_pipeline
from src.utils.http import HTTPException

logger = logging.getLogger(__name__)


async def start_run(request: web.Request) -> web.Response:
    """
    ---
    summary: Start a new video processing run
    description: |
      Creates a new video processing run with the specified RTSP source and captioning parameters.
      Returns information about the created run including WebRTC peer ID for video streaming.
    tags:
      - Runs
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            required:
              - rtspUrl
            properties:
              rtspUrl:
                type: string
                description: RTSP URL of the video source
                example: "rtsp://localhost:8554/stream"
              prompt:
                type: string
                default: "Describe what you see in the image in one sentence."
                description: Prompt for the captioning model
              modelName:
                type: string
                default: "OpenGVLab/InternVL2-2B"
                description: Name of the captioning model to use
              maxNewTokens:
                type: integer
                minimum: 1
                maximum: 4096
                default: 70
                description: Maximum number of tokens to generate
              pipelineName:
                type: string
                description: Optional pipeline name override
    responses:
      "200":
        description: Run information
        content:
          application/json:
            schema:
              type: object
              properties:
                runId:
                  type: string
                  description: Unique run identifier
                pipelineId:
                  type: string
                  description: Pipeline server's identifier
                peerId:
                  type: string
                  description: WebRTC peer ID
                metadataFile:
                  type: string
                  description: Path to metadata output file
      "400":
        description: Invalid request
      "502":
        description: Pipeline server error
    """
    # Parse and validate request
    try:
        data = await request.json()
        req = StartRunRequest.from_dict(data)
    except json.JSONDecodeError:
        logger.error("start_run: Invalid JSON in request body")
        return web.json_response({"error": "Invalid JSON"}, status=400)
    except ValueError as e:
        logger.error(f"start_run: Validation error - {e}")
        return web.json_response({"error": str(e)}, status=400)
    
    # Generate run identifiers
    run_id = uuid.uuid4().hex[:10]
    peer_id = f"stream-{run_id}"
    metadata_file = f"/tmp/results-{run_id}.jsonl"
    pipeline_name = (req.pipelineName or PIPELINE_NAME).strip() or PIPELINE_NAME
    
    logger.info(
        f"start_run: Starting pipeline '{pipeline_name}' with run_id={run_id}, "
        f"rtspUrl={req.rtspUrl}, model={req.modelName}"
    )

    # Start the pipeline
    session = request.app["http_session"]
    prompt = (req.prompt or "").strip() or "Describe what you see in the image in one sentence."
    model_name = (req.modelName or "").strip() or "OpenGVLab/InternVL2-2B"
    
    try:
        pipeline_id = await start_pipeline(
            session=session,
            pipeline_name=pipeline_name,
            source_uri=req.rtspUrl,
            peer_id=peer_id,
            metadata_file=metadata_file,
            prompt=prompt,
            model_name=model_name,
            max_new_tokens=req.maxNewTokens,
        )
    except HTTPException as e:
        logger.error(f"start_run: Pipeline server error - status={e.status_code}, detail={e.detail}")
        return web.json_response(e.detail, status=e.status_code)
    
    # Store run info
    info = RunInfo(
        runId=run_id,
        pipelineId=pipeline_id,
        peerId=peer_id,
        metadataFile=metadata_file,
        modelName=model_name,
        pipelineName=pipeline_name,
    )
    runs: dict[str, RunInfo] = request.app["runs"]
    runs[info.runId] = info
    
    logger.info(
        f"start_run: Successfully started run {info.runId} with pipeline_id={pipeline_id}, "
        f"total active runs: {len(runs)}"
    )
    return web.json_response(info.to_dict())


async def list_runs(request: web.Request) -> web.Response:
    """
    ---
    summary: List all active runs
    description: Returns a list of all currently active video processing runs.
    tags:
      - Runs
    responses:
      "200":
        description: List of active runs
        content:
          application/json:
            schema:
              type: array
              items:
                type: object
                properties:
                  runId:
                    type: string
                  pipelineId:
                    type: string
                  peerId:
                    type: string
                  metadataFile:
                    type: string
    """
    runs: dict[str, RunInfo] = request.app["runs"]
    return web.json_response([info.to_dict() for info in runs.values()])


async def get_run(request: web.Request) -> web.Response:
    """
    ---
    summary: Get run details
    description: Returns details about a specific video processing run.
    tags:
      - Runs
    parameters:
      - name: run_id
        in: path
        required: true
        schema:
          type: string
        description: The run ID
    responses:
      "200":
        description: Run information
        content:
          application/json:
            schema:
              type: object
              properties:
                runId:
                  type: string
                pipelineId:
                  type: string
                peerId:
                  type: string
                metadataFile:
                  type: string
      "404":
        description: Run not found
    """
    run_id = request.match_info["run_id"]
    runs: dict[str, RunInfo] = request.app["runs"]
    info = runs.get(run_id)
    
    if not info:
        return web.json_response({"message": "Run not found"}, status=404)
    return web.json_response(info.to_dict())


async def stop_run_handler(request: web.Request) -> web.Response:
    """
    ---
    summary: Stop a run
    description: Stops a video processing run and cleans up resources.
    tags:
      - Runs
    parameters:
      - name: run_id
        in: path
        required: true
        schema:
          type: string
        description: The run ID to stop
    responses:
      "200":
        description: Run stopped successfully
        content:
          application/json:
            schema:
              type: object
              properties:
                status:
                  type: string
                  example: "stopped"
                runId:
                  type: string
      "404":
        description: Run not found
      "502":
        description: Pipeline server error
    """
    run_id = request.match_info["run_id"]
    runs: dict[str, RunInfo] = request.app["runs"]
    info = runs.get(run_id)
    
    if not info:
        return web.json_response({"message": "Run not found"}, status=404)
    
    session = request.app["http_session"]
    logger.info(f"stop_run: Stopping run {run_id} with pipeline_id={info.pipelineId}")
    
    try:
        await stop_pipeline(session, info.pipelineId)
    except HTTPException as e:
        # Remove from local tracking even on error to allow UI cleanup
        runs.pop(run_id, None)
        logger.error(f"stop_run: Pipeline server error - {e.detail}")
        return web.json_response(e.detail, status=e.status_code)
    
    # Cleanup
    runs.pop(run_id, None)
    try:
        Path(info.metadataFile).unlink(missing_ok=True)
    except OSError:
        pass
    
    logger.info(f"stop_run: Run {run_id} stopped successfully, remaining runs: {len(runs)}")
    return web.json_response({"status": "stopped", "runId": run_id})


# Route definitions for registration with Swagger
runs_routes = [
    web.post("/api/runs", start_run),
    web.get("/api/runs", list_runs, allow_head=False),
    web.get("/api/runs/{run_id}", get_run, allow_head=False),
    web.delete("/api/runs/{run_id}", stop_run_handler),
]
