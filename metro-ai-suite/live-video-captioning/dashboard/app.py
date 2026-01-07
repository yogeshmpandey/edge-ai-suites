import asyncio
import json
import os
import uuid
from pathlib import Path
from typing import Any, AsyncGenerator, Optional

from fastapi import FastAPI, HTTPException, Response
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from urllib import request as urllib_request
from urllib.error import HTTPError, URLError

# Import metrics router
from metrics import router as metrics_router

APP_PORT = int(os.environ.get("DASHBOARD_PORT", "4173"))
METADATA_FILE = os.environ.get("METADATA_FILE", "/tmp/results.jsonl")
PEER_ID = os.environ.get("WEBRTC_PEER_ID", "genai_pipeline")
SIGNALING_URL = os.environ.get("SIGNALING_URL", "http://localhost:8889")
POLL_INTERVAL = float(os.environ.get("METADATA_POLL_SECONDS", "1"))

PIPELINE_SERVER_URL = os.environ.get(
    "PIPELINE_SERVER_URL", "http://video-ingestion:8080"
)
PIPELINE_NAME = os.environ.get("PIPELINE_NAME", "genai_pipeline")

BASE_DIR = Path(__file__).parent
MODELS_DIR = Path(os.environ.get("MODELS_DIR", str(BASE_DIR / "ov_models")))
PUBLIC_DIR = BASE_DIR / "public"

app = FastAPI()

# Include metrics WebSocket routes
app.include_router(metrics_router)


class StartRunRequest(BaseModel):
    rtspUrl: str = Field(..., min_length=1)
    prompt: str = Field(default="Describe what you see in the image in one sentence.")
    modelName: str = Field(default="OpenGVLab/InternVL2-2B")
    maxNewTokens: int = Field(default=70, ge=1, le=4096)
    pipelineName: Optional[str] = Field(default=None)


class RunInfo(BaseModel):
    runId: str
    pipelineId: str
    peerId: str
    metadataFile: str
    modelName: Optional[str] = None
    pipelineName: Optional[str] = None


class ModelList(BaseModel):
    models: list[str]


class PipelineList(BaseModel):
    pipelines: list[str]


RUNS: dict[str, RunInfo] = {}


def _discover_models(root: Path) -> list[str]:
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


def _discover_pipelines_remote() -> list[str]:
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines"
    try:
        raw = _http_json("GET", url)
        payload = json.loads(raw)
        # Accept either list[str] or list[dict {name}] or {'pipelines': [...]}
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
                    elif isinstance(item, dict) and isinstance(
                        item.get("version"), str
                    ):
                        names.append(item["version"])
                return names or [PIPELINE_NAME]
    except Exception:
        return [PIPELINE_NAME]
    return [PIPELINE_NAME]


def _read_latest_line(path: Path) -> Optional[str]:
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8") as handle:
            lines = [line.strip() for line in handle if line.strip()]
    except OSError:
        return None
    if not lines:
        return None
    return lines[-1]


def _http_json(method: str, url: str, payload: Optional[dict[str, Any]] = None) -> str:
    headers = {
        "Accept": "application/json",
    }
    data = None
    if payload is not None:
        body = json.dumps(payload).encode("utf-8")
        data = body
        headers["Content-Type"] = "application/json"
    req = urllib_request.Request(url=url, data=data, headers=headers, method=method)
    try:
        with urllib_request.urlopen(req, timeout=120) as resp:
            return resp.read().decode("utf-8")
    except HTTPError as err:
        details = None
        try:
            details = err.read().decode("utf-8")
        except Exception:
            details = None
        raise HTTPException(
            status_code=502,
            detail={
                "message": "Pipeline server error",
                "status": err.code,
                "body": details,
            },
        )
    except URLError as err:
        raise HTTPException(
            status_code=502,
            detail={"message": "Pipeline server unreachable", "error": str(err)},
        )


@app.get("/runtime-config.js")
async def runtime_config() -> Response:
    payload = {
        "signalingUrl": SIGNALING_URL,
        "defaultPeerId": PEER_ID,
        "defaultMetadataFile": METADATA_FILE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return Response(content=body, media_type="application/javascript")


@app.get("/api/models", response_model=ModelList)
async def list_models() -> ModelList:
    models = _discover_models(MODELS_DIR)
    return ModelList(models=models)


@app.get("/api/pipelines", response_model=PipelineList)
async def list_pipelines() -> PipelineList:
    names = _discover_pipelines_remote()
    return PipelineList(pipelines=names)


@app.post("/api/runs")
async def start_run(req: StartRunRequest) -> RunInfo:
    run_id = uuid.uuid4().hex
    peer_id = f"stream-{run_id[:10]}"
    metadata_file = f"/tmp/results-{run_id[:10]}.jsonl"

    pipeline_name = (req.pipelineName or PIPELINE_NAME).strip() or PIPELINE_NAME

    start_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/user_defined_pipelines/{pipeline_name}"
    payload = {
        "source": {"uri": req.rtspUrl, "type": "uri"},
        "destination": {
            "metadata": {"type": "file", "path": metadata_file, "format": "json-lines"},
            "frame": {"type": "webrtc", "peer-id": peer_id, "bitrate": 5000},
        },
        "parameters": {
            "captioner-prompt": (req.prompt or "").strip()
            or "Describe what you see in the image in one sentence.",
            "captioner_model_name": (req.modelName or "").strip()
            or "OpenGVLab/InternVL2-2B",
            "captioner_max_new_tokens": req.maxNewTokens,
        },
    }

    raw = _http_json("POST", start_url, payload=payload)
    pipeline_id = raw.replace('"', "").strip()
    if not pipeline_id:
        raise HTTPException(
            status_code=502,
            detail={
                "message": "Pipeline server returned empty pipeline id",
                "body": raw,
            },
        )

    model_name = (req.modelName or "").strip() or "InternVL2-2B"
    info = RunInfo(
        runId=run_id[:10],
        pipelineId=pipeline_id,
        peerId=peer_id,
        metadataFile=metadata_file,
        modelName=model_name,
        pipelineName=pipeline_name,
    )
    RUNS[info.runId] = info
    return info


@app.get("/api/runs")
async def list_runs() -> list[RunInfo]:
    return list(RUNS.values())


async def _multiplexed_metadata_generator() -> AsyncGenerator[str, None]:
    """Generator that reads metadata from all active runs and multiplexes into a single SSE stream."""
    last_payloads: dict[str, str] = {}
    last_modified_times: dict[str, float] = {}

    while True:
        try:
            # Get current list of runs (defensive copy)
            current_runs = dict(RUNS)

            for run_id, info in current_runs.items():
                path = Path(info.metadataFile)

                # Check if file exists and get its modification time
                if not path.exists():
                    continue

                try:
                    current_mtime = path.stat().st_mtime
                    last_mtime = last_modified_times.get(run_id, 0)

                    # Only read file if it was modified since last check
                    if current_mtime > last_mtime:
                        latest = _read_latest_line(path)
                        if latest and latest != last_payloads.get(run_id):
                            last_payloads[run_id] = latest
                            last_modified_times[run_id] = current_mtime

                            # Wrap the data with runId for client-side demultiplexing
                            try:
                                data_obj = json.loads(latest)
                                envelope = {"runId": run_id, "data": data_obj}
                            except json.JSONDecodeError:
                                envelope = {"runId": run_id, "data": latest}

                            yield f"data: {json.dumps(envelope)}\n\n"
                except OSError:
                    # File might have been deleted or is inaccessible
                    continue

            # Send heartbeat to keep connection alive
            yield f": heartbeat\n\n"

            # Clean up stale entries
            current_run_ids = set(current_runs.keys())
            stale_ids = [rid for rid in last_payloads if rid not in current_run_ids]
            for rid in stale_ids:
                last_payloads.pop(rid, None)
                last_modified_times.pop(rid, None)

        except Exception as e:
            # Log error but don't break the generator
            print(f"Error in multiplexed metadata generator: {e}")
            yield f": error - {e}\n\n"

        await asyncio.sleep(POLL_INTERVAL)


@app.get("/api/runs/metadata-stream")
async def multiplexed_metadata_stream() -> StreamingResponse:
    """Multiplexed SSE stream that provides metadata for all active runs."""
    print("Multiplexed metadata stream requested")
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


@app.get("/api/runs/{run_id}")
async def get_run(run_id: str) -> RunInfo:
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})
    return info


@app.delete("/api/runs/{run_id}")
async def stop_run(run_id: str) -> dict[str, str]:
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})
    stop_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/{info.pipelineId}"
    _http_json("DELETE", stop_url)
    RUNS.pop(run_id, None)
    try:
        Path(info.metadataFile).unlink(missing_ok=True)
    except OSError:
        pass
    return {"status": "stopped", "runId": run_id}


@app.get("/")
async def root() -> FileResponse:
    return FileResponse(PUBLIC_DIR / "index.html")


app.mount("/", StaticFiles(directory=PUBLIC_DIR, html=True), name="public")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("app:app", host="0.0.0.0", port=APP_PORT, reload=True)
