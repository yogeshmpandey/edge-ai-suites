# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import asyncio
import json
import logging
import re
import uuid
from typing import AsyncGenerator, Optional
from urllib.parse import quote
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from ..config import (
    PIPELINE_NAME,
    PIPELINE_SERVER_URL,
    MQTT_TOPIC_PREFIX,
    WEBRTC_BITRATE,
    ENABLE_EMBEDDING,
    NPU_FORCED_RESOLUTION,
)
from ..models import RunInfo, StartRunRequest
from ..models.requests import DEFAULT_PROMPT
from ..services import (
    discover_pipelines_remote,
    http_json,
    get_mqtt_subscriber,
    get_pipeline_state,
)
from ..state import RUNS

router = APIRouter(prefix="/api", tags=["captions"])
logger = logging.getLogger("app.runs")
WEBRTC_PEER_ID_MAX_LENGTH = 8
WEBRTC_PEER_ID_PREFIX = "s"
DEFAULT_RESOLUTION_SUFFIX = "_Default_Resolution"
TARGET_VLM_OVERRIDE_PIPELINES = {
    "GenAI_RTSP_Pipeline_Software",
    "GenAI_RTSP_Pipeline_Software_Default_Resolution",
    "GenAI_RTSP_Pipeline_Hardware",
    "GenAI_RTSP_Pipeline_Hardware_Default_Resolution",
    "GenAI_Camera_Pipeline_Software",
    "GenAI_Camera_Pipeline_Software_Default_Resolution",
    "GenAI_Camera_Pipeline_Hardware",
    "GenAI_Camera_Pipeline_Hardware_Default_Resolution",
    "GenAI_Detection_RTSP_Pipeline_Software",
    "GenAI_Detection_RTSP_Pipeline_Hardware",
    "GenAI_Camera_Detection_Pipeline_Software",
    "GenAI_Camera_Detection_Pipeline_Hardware",
}
CPU_SCHEDULER_CONFIG = "max_num_batched_tokens=256,cache_size=4,enable_prefix_caching=true,dynamic_split_fuse=true,use_cache_eviction=true"
GPU_SCHEDULER_CONFIG = "max_num_batched_tokens=512,cache_size=8,enable_prefix_caching=true,dynamic_split_fuse=true,use_cache_eviction=true"

# Pipeline states reported by the DL Streamer pipeline server that are considered
# healthy while a stream is starting up. ``running`` means frames are flowing;
# ``queued`` means the instance is scheduled and about to run. Any other state
# (error, aborted, completed, …) means the stream has failed to come up.
_HEALTHY_PIPELINE_STATES = {"running", "queued"}


def _is_linux_video_device(source_uri: str) -> bool:
    source = (source_uri or "").strip()
    return source.startswith("/dev/video") and source[len("/dev/video") :].isdigit()


def _is_camera_pipeline_name(pipeline_name: str) -> bool:
    """Best-effort check for camera-capable pipeline identifiers."""
    return "camera" in (pipeline_name or "").strip().lower()


def _resolve_pipeline_name(requested_pipeline: str) -> str:
    """Resolve requested pipeline name against discovered pipeline identifiers."""
    normalized = (requested_pipeline or "").strip()
    if not normalized:
        return PIPELINE_NAME

    discovered = discover_pipelines_remote()
    allowed_names = {
        (item.get("pipeline_name") or "").strip()
        for item in discovered
        if isinstance(item, dict)
    }
    # The /api/pipelines response intentionally hides proxy pipelines ending with
    # _Default_Resolution. Accept those internal aliases only for discovered base names.
    allowed_names.update(
        {
            f"{name}{DEFAULT_RESOLUTION_SUFFIX}"
            for name in allowed_names
            if name and not name.endswith(DEFAULT_RESOLUTION_SUFFIX)
        }
    )
    for allowed_name in allowed_names:
        if allowed_name == normalized:
            return allowed_name

    raise HTTPException(
        status_code=400,
        detail={
            "message": f"Unknown pipelineName '{normalized}'. Choose one from /api/pipelines.",
        },
    )


# TODO: This is a temporary workaround to force lower resolution for NPU runs that don't support higher resolutions.
# This requires changes to the dlstreamer pipeline server API and will remove once the fix is available.
def _normalize_pipeline_name_for_vlm_device(pipeline_name: str, vlm_device: str) -> str:
    """Normalize pipeline aliases based on VLM device constraints.

    NPU runs must avoid internal *_Default_Resolution aliases because NPU
    resolution is enforced internally by backend parameters.
    """
    if (vlm_device or "").strip().lower() != "npu":
        return pipeline_name
    if not (pipeline_name or "").endswith(DEFAULT_RESOLUTION_SUFFIX):
        return pipeline_name
    return pipeline_name[: -len(DEFAULT_RESOLUTION_SUFFIX)]



def _sanitize_run_name(run_name: str) -> str:
    """Normalize a user-supplied run name into a safe run identifier."""
    sanitized = re.sub(r"\s+", "_", run_name.strip())
    return re.sub(r"[^a-zA-Z0-9_-]", "", sanitized)


def _build_unique_run_name(requested_name: Optional[str]) -> Optional[str]:
    """Return a sanitized, unique run name or None when no valid name was provided."""
    if not requested_name or not requested_name.strip():
        return None

    sanitized = _sanitize_run_name(requested_name)
    if not sanitized:
        return None

    run_name = sanitized
    counter = 1
    while run_name in RUNS:
        run_name = f"{sanitized}_{counter}"
        counter += 1

    return run_name


def _generate_peer_id() -> str:
    """Generate a short, unique WebRTC peer ID accepted by the pipeline server."""
    existing_peer_ids = {run.peerId for run in RUNS.values()}
    peer_body_length = WEBRTC_PEER_ID_MAX_LENGTH - len(WEBRTC_PEER_ID_PREFIX)
    if peer_body_length < 1:
        raise RuntimeError("Invalid WebRTC peer ID configuration")

    while True:
        candidate = f"{WEBRTC_PEER_ID_PREFIX}{uuid.uuid4().hex[:peer_body_length]}"
        if candidate not in existing_peer_ids:
            return candidate


def _build_pipeline_parameters(req: StartRunRequest, run_id: str, pipeline_name: str) -> dict:
    selected_vlm_device = (req.vlmDevice or "").strip().lower()
    selected_detection_device = (req.detectionDevice or "").strip().lower()
    detection_model_name = (req.detectionModelName or "").strip() or "yolov8s"

    parameters = {
        "mqtt_publisher": {
            "topic": f"{MQTT_TOPIC_PREFIX}/{run_id}",
            "publish_frame": bool(
                ENABLE_EMBEDDING
            ),  # Only publish frames if embedding is enabled
        },
    }

    # TODO: This is a temporary workaround to force lower resolution for NPU runs that don't support higher resolutions.
    # This requires changes to the dlstreamer pipeline server API and will remove once the fix is available.
    is_npu_pipeline = selected_vlm_device == "npu"

    if is_npu_pipeline:
        frame_width = NPU_FORCED_RESOLUTION
        frame_height = NPU_FORCED_RESOLUTION
        logger.debug(
            f"NPU device selected: forcing resolution to {NPU_FORCED_RESOLUTION}x{NPU_FORCED_RESOLUTION}"
        )
    else:
        frame_width = req.frameWidth
        frame_height = req.frameHeight

    optional_parameters = {
        "frame_rate": req.frameRate,
        "frame_width": frame_width,
        "frame_height": frame_height,
    }
    parameters.update(
        {key: value for key, value in optional_parameters.items() if value is not None}
    )

    if req.chunkSize is not None:
        parameters["queue_size"] = max(1, req.chunkSize)

    if req.captionerProperties:
        # Forward gvagenai element-properties as a dedicated pipeline parameter.
        captioner_properties = dict(req.captionerProperties)
        # NPU captioner initialization fails when scheduler-config is injected.
        if is_npu_pipeline:
            captioner_properties.pop("scheduler-config", None)
        if captioner_properties:
            parameters["captioner-properties"] = captioner_properties

    # Detection device / pre-process-backend are substituted directly into the
    # gvadetect element in the pipeline string (caps-affecting properties must be
    # set before gst_parse_launch links the pipeline, otherwise hardware pipelines
    # fail to link gvadetect to the VAMemory queue).
    if selected_detection_device in {"cpu", "gpu", "npu"}:
        # Hardware detection pipelines run gvadetect on VAMemory surfaces, which the
        # CPU pre-process backend cannot consume; Software detection pipelines run on
        # SYSTEM memory and only support CPU. Reject incompatible combinations so the
        # pipeline does not fail later with an opaque gst_parse link error.
        is_detection_pipeline = "Detection" in pipeline_name
        if is_detection_pipeline:
            if pipeline_name.endswith("_Hardware") and selected_detection_device == "cpu":
                raise HTTPException(
                    status_code=400,
                    detail={
                        "message": (
                            f"Detection device 'cpu' is not supported by hardware "
                            f"detection pipeline '{pipeline_name}'. Use 'gpu' or 'npu'."
                        ),
                    },
                )
            if pipeline_name.endswith("_Software") and selected_detection_device != "cpu":
                raise HTTPException(
                    status_code=400,
                    detail={
                        "message": (
                            f"Detection device '{selected_detection_device}' is not "
                            f"supported by software detection pipeline '{pipeline_name}'. "
                            "Use 'cpu'."
                        ),
                    },
                )
        pre_process_backend = {
            "cpu": "opencv",
            "gpu": "va-surface-sharing",
            "npu": "va",
        }[selected_detection_device]
        parameters.update(
            {
                "detection_model_name": detection_model_name,
                "detection_threshold": req.detectionThreshold,
                "detection_device": selected_detection_device.upper(),
                "detection_pre_process_backend": pre_process_backend,
            }
        )

    if (
        selected_vlm_device in {"cpu", "gpu", "npu"}
        and pipeline_name in TARGET_VLM_OVERRIDE_PIPELINES
    ):
        model_name = (req.modelName or "").strip() or "OpenGVLab/InternVL2-2B"
        prompt = (req.prompt or "").strip() or DEFAULT_PROMPT

        captioner_properties: dict[str, object] = {
            "device": selected_vlm_device.upper(),
            "model-path": f"/home/pipeline-server/models/{model_name}",
            "prompt": prompt,
            "chunk-size": req.chunkSize or 1,
        }

        if selected_vlm_device == "npu":
            captioner_properties["generation-config"] = (
                f"max_new_tokens={req.maxNewTokens},num_beams=1,do_sample=false,"
                "temperature=0.1,repetition_penalty=1.1,MAX_PROMPT_LEN=4096"
            )
            # NPU/VLM must not receive scheduler configuration.
            captioner_properties.pop("scheduler-config", None)

        scheduler_config = None
        if selected_vlm_device == "cpu":
            scheduler_config = CPU_SCHEDULER_CONFIG
        elif selected_vlm_device == "gpu":
            scheduler_config = GPU_SCHEDULER_CONFIG

        if scheduler_config is not None:
            # Element-properties use hyphenated GStreamer property names.
            captioner_properties["scheduler-config"] = scheduler_config

        if selected_vlm_device == "gpu":
            captioner_properties["model-cache-path"] = "/tmp/ov_cache"

        parameters["captioner-properties"] = captioner_properties

    return parameters


def _build_start_payload(
    req: StartRunRequest, run_id: str, peer_id: str, pipeline_name: str
) -> dict:
    source_uri = (req.rtspUrl or "").strip()
    if _is_linux_video_device(source_uri):
        source = {"device": source_uri, "type": "webcam"}
    else:
        source = {"uri": source_uri, "type": "uri"}

    frame_destination = {
        "type": "webrtc",
        "peer-id": peer_id,
        "bitrate": WEBRTC_BITRATE,
    }
    # Keep previous default behavior (no bounding boxes) unless explicitly enabled.
    if not bool(req.includeRoiBoundingBox):
        frame_destination["overlay"] = False

    return {
        "source": source,
        "destination": {
            "frame": frame_destination,
        },
        "parameters": _build_pipeline_parameters(req, run_id, pipeline_name),
    }


def _extract_pipeline_id(raw: str) -> str:
    pipeline_id = raw.replace('"', "").strip()
    if not pipeline_id:
        raise HTTPException(
            status_code=502,
            detail={
                "message": "Pipeline server returned empty pipeline id",
                "body": raw,
            },
        )
    return pipeline_id


@router.post("/generate_captions_alerts")
async def start_run(req: StartRunRequest) -> RunInfo:
    """Start a new video captioning run and generate captions and alerts."""
    requested_source = (req.rtspUrl or "").strip()
    requested_pipeline = (req.pipelineName or "").strip()
    using_camera_source = _is_linux_video_device(requested_source)

    pipeline_name = _resolve_pipeline_name(requested_pipeline)
    pipeline_name = _normalize_pipeline_name_for_vlm_device(
        pipeline_name, (req.vlmDevice or "")
    )
    if using_camera_source and not _is_camera_pipeline_name(pipeline_name):
        if requested_pipeline:
            detail_message = (
                f"Pipeline '{pipeline_name}' is not camera-compatible. "
                "Use a camera pipeline for /dev/videoN sources."
            )
        else:
            detail_message = (
                "Camera sources require a camera-compatible pipelineName when the "
                "default PIPELINE_NAME is not camera-compatible."
            )
        raise HTTPException(status_code=400, detail={"message": detail_message})

    if using_camera_source:
        for existing in RUNS.values():
            if (
                existing.rtspUrl or ""
            ).strip() == requested_source and existing.status == "running":
                raise HTTPException(
                    status_code=409,
                    detail={
                        "message": f"Camera device {requested_source} is already in use by run {existing.runId}",
                    },
                )

    run_name = _build_unique_run_name(req.runName)

    # Use runName for run_id if provided, otherwise generate UUID
    if run_name:
        run_id = run_name
    else:
        run_id = uuid.uuid4().hex[:10]

    peer_id = _generate_peer_id()

    # MQTT topic for this run's metadata
    mqtt_topic = f"{MQTT_TOPIC_PREFIX}"

    encoded_pipeline_name = quote(pipeline_name, safe="")
    start_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/user_defined_pipelines/{encoded_pipeline_name}"
    payload = _build_start_payload(req, run_id, peer_id, pipeline_name)

    logger.debug("Starting caption pipeline request")

    raw = http_json("POST", start_url, payload=payload)
    pipeline_id = _extract_pipeline_id(raw)

    model_name = (req.modelName or "").strip() or "InternVL2-2B"
    # Use full run_id for custom names, truncated for UUID-based
    final_run_id = run_id if run_name else run_id[:10]
    info = RunInfo(
        runId=final_run_id,
        pipelineId=pipeline_id,
        peerId=peer_id,
        mqttTopic=mqtt_topic,
        modelName=model_name,
        vlmDevice=((req.vlmDevice or "").strip().lower() or None),
        detectionDevice=((req.detectionDevice or "").strip().lower() or None),
        pipelineName=pipeline_name,
        runName=run_name,
        prompt=(req.prompt or "").strip() or DEFAULT_PROMPT,
        maxTokens=req.maxNewTokens,
        rtspUrl=req.rtspUrl,
        frameRate=req.frameRate,
        chunkSize=req.chunkSize,
        frameWidth=req.frameWidth,
        frameHeight=req.frameHeight,
    )
    RUNS[info.runId] = info
    return info


@router.get("/generate_captions_alerts")
async def list_runs() -> list[RunInfo]:
    """List all active captioning runs."""
    return list(RUNS.values())


async def _multiplexed_metadata_generator() -> AsyncGenerator[str, None]:
    """Generator that receives metadata from MQTT and multiplexes into a single SSE stream.

    A status heartbeat is sent every second when no MQTT message arrives, carrying
    the current status of every active run so the frontend can react when a run
    transitions to ``"error"`` (detected by the background health monitor).
    """
    message_queue: asyncio.Queue = asyncio.Queue()
    subscribed_runs: set[str] = set()

    def on_message(run_id: str, data: dict, received_at: float):
        """Callback for MQTT messages - puts them into the async queue."""
        try:
            asyncio.get_event_loop().call_soon_threadsafe(
                message_queue.put_nowait, (run_id, data, received_at)
            )
        except Exception as e:
            logger.error(f"Error queueing MQTT message: {e}")

    mqtt_subscriber = await get_mqtt_subscriber()
    try:

        while True:
            try:
                # Update subscriptions based on current active runs
                current_runs = set(RUNS.keys())

                # Subscribe to new runs
                new_runs = current_runs - subscribed_runs
                for run_id in new_runs:
                    mqtt_subscriber.subscribe_to_run(run_id, on_message)
                    subscribed_runs.add(run_id)
                    logger.info(f"Subscribed to MQTT topic for run {run_id}")

                # Unsubscribe from stopped runs
                stopped_runs = subscribed_runs - current_runs
                for run_id in stopped_runs:
                    mqtt_subscriber.unsubscribe_from_run(run_id)
                    subscribed_runs.discard(run_id)
                    logger.info(f"Unsubscribed from MQTT topic for run {run_id}")

                # Process any messages in the queue with a short timeout
                try:
                    run_id, data, received_at = await asyncio.wait_for(
                        message_queue.get(), timeout=1.0
                    )

                    # Wrap the data with runId for client-side demultiplexing
                    envelope = {
                        "runId": run_id,
                        "data": data,
                        "received_at": received_at,
                    }
                    yield f"data: {json.dumps(envelope)}\n\n"

                except asyncio.TimeoutError:
                    # No MQTT message – send a status heartbeat so the frontend
                    # learns when a run transitions to "error".
                    status_payload = {
                        "type": "status",
                        "runs": {rid: info.status for rid, info in RUNS.items()},
                    }
                    yield f"data: {json.dumps(status_payload)}\n\n"

            except Exception as e:
                logger.error(f"Error in multiplexed metadata generator: {e}")
                yield ": error\n\n"
                await asyncio.sleep(1)

    finally:
        # Reuse the already-resolved subscriber — avoids creating a new connection
        # during app shutdown when the global subscriber may already be torn down.
        for run_id in subscribed_runs:
            mqtt_subscriber.unsubscribe_from_run(run_id)
        logger.info("Cleaned up MQTT subscriptions")


@router.get("/generate_captions_alerts/metadata-stream")
async def multiplexed_metadata_stream() -> StreamingResponse:
    """Multiplexed SSE stream that provides captions and alerts metadata for all active runs."""
    logger.info("Multiplexed metadata stream requested")
    headers = {
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Headers": "Cache-Control",
    }
    return StreamingResponse(
        _multiplexed_metadata_generator(),
        media_type="text/event-stream",
        headers=headers,
    )


@router.get("/generate_captions_alerts/{run_id}/stream-ready")
async def stream_ready(run_id: str) -> dict[str, object]:
    """Report whether the WebRTC stream for a run is publishing yet.

    The DL Streamer pipeline needs a few seconds after start before it begins
    publishing frames to mediamtx. The UI polls this endpoint and only loads the
    video iframe once frames are flowing, avoiding mediamtx's "stream not
    found, retrying" page.

    This endpoint is the backend half of a two-stage readiness gate: the
    backend answers "is the pipeline alive and producing?", mediamtx answers
    "can the browser watch it yet?" (the UI confirms the latter with a WHEP
    probe before loading the iframe).

    Readiness is derived from the pipeline server alone: the run is ready when
    its instance is ``RUNNING`` and reports a positive ``avg_fps`` — frames
    moving through the pipeline are being published to mediamtx by the WebRTC
    sink. This deliberately avoids the mediamtx control API, so mediamtx can
    run with its API disabled. The pipeline state also lets the UI fail fast
    when the instance leaves the ``RUNNING``/``QUEUED`` states (or vanishes)
    instead of staying stuck on "Connecting…".
    """
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})

    reachable, state, avg_fps = await asyncio.to_thread(
        get_pipeline_state, info.pipelineId
    )

    # Pipeline server temporarily unreachable – treat as "still starting" and
    # let the UI keep waiting; the health monitor handles persistent outages.
    if not reachable:
        return {
            "runId": run_id,
            "peerId": info.peerId,
            "ready": False,
            "state": None,
            "error": False,
        }

    # The instance has vanished or entered a non-healthy state (error, aborted,
    # completed, …). The stream will never come up – report a hard error.
    if state is None or state not in _HEALTHY_PIPELINE_STATES:
        info.status = "error"
        return {
            "runId": run_id,
            "peerId": info.peerId,
            "ready": False,
            "state": state,
            "error": True,
        }

    # Healthy: ready once frames are flowing; otherwise keep waiting.
    return {
        "runId": run_id,
        "peerId": info.peerId,
        "ready": state == "running" and avg_fps > 0,
        "state": state,
        "error": False,
    }


@router.get("/generate_captions_alerts/{run_id}")
async def get_run(run_id: str) -> RunInfo:
    """Get details of a specific captioning run."""
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})
    return info


@router.delete("/generate_captions_alerts/{run_id}")
async def stop_run(run_id: str) -> dict[str, str]:
    """Stop a running captioning pipeline."""
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})
    stop_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/{info.pipelineId}"

    # Try to stop pipeline on backend, but always remove from internal list
    # A failure (502) usually means the pipeline is already stopped
    try:
        http_json("DELETE", stop_url)
    except Exception:
        # Pipeline may already be stopped or unreachable - continue cleanup
        pass

    RUNS.pop(run_id, None)
    return {"status": "stopped", "runId": run_id}
