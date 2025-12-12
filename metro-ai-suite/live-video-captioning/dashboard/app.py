import asyncio
import json
import os
import uuid
from pathlib import Path
from typing import Any, AsyncGenerator, Optional

import psutil
from fastapi import FastAPI, HTTPException, Response
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from urllib import request as urllib_request
from urllib.error import HTTPError, URLError

APP_PORT = int(os.environ.get("DASHBOARD_PORT", "4173"))
METADATA_FILE = os.environ.get("METADATA_FILE", "/tmp/results.jsonl")
PEER_ID = os.environ.get("WEBRTC_PEER_ID", "genai_pipeline")
SIGNALING_URL = os.environ.get("SIGNALING_URL", "http://localhost:8889")
POLL_INTERVAL = float(os.environ.get("METADATA_POLL_SECONDS", "1"))

PIPELINE_SERVER_URL = os.environ.get("PIPELINE_SERVER_URL", "http://video-ingestion:8080")
PIPELINE_NAME = os.environ.get("PIPELINE_NAME", "genai_pipeline")

BASE_DIR = Path(__file__).parent
PUBLIC_DIR = BASE_DIR / "public"

app = FastAPI()


class StartRunRequest(BaseModel):
    rtspUrl: str = Field(..., min_length=1)
    prompt: str = Field(default="Describe what you see in the image in one sentence.")
    modelName: str = Field(default="OpenGVLab/InternVL2-2B")
    maxNewTokens: int = Field(default=70, ge=1, le=4096)


class RunInfo(BaseModel):
    runId: str
    pipelineId: str
    peerId: str
    metadataFile: str


RUNS: dict[str, RunInfo] = {}


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


async def _metadata_generator(path: Path) -> AsyncGenerator[str, None]:
    last_payload = None
    while True:
        latest = _read_latest_line(path)
        if latest and latest != last_payload:
            last_payload = latest
            yield f"data: {latest}\n\n"
        await asyncio.sleep(POLL_INTERVAL)


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
        with urllib_request.urlopen(req, timeout=30) as resp:
            return resp.read().decode("utf-8")
    except HTTPError as err:
        details = None
        try:
            details = err.read().decode("utf-8")
        except Exception:
            details = None
        raise HTTPException(status_code=502, detail={"message": "Pipeline server error", "status": err.code, "body": details})
    except URLError as err:
        raise HTTPException(status_code=502, detail={"message": "Pipeline server unreachable", "error": str(err)})


async def _system_stats_generator() -> AsyncGenerator[str, None]:
    while True:
        cpu = await asyncio.to_thread(psutil.cpu_percent, interval=0.5)
        mem = psutil.virtual_memory()
        # GPU stats placeholder for Intel iGPU; set to None/0 if unavailable.
        stats = {
            "cpu_percent": cpu,
            "mem_total_gb": round(mem.total / (1024**3), 2),
            "mem_used_gb": round(mem.used / (1024**3), 2),
            "mem_percent": mem.percent,
            "gpu_percent": None,
            "vram_total_gb": None,
            "vram_used_gb": None,
        }
        yield f"data: {json.dumps(stats)}\n\n"
        await asyncio.sleep(POLL_INTERVAL)


@app.get("/runtime-config.js")
async def runtime_config() -> Response:
    payload = {
        "signalingUrl": SIGNALING_URL,
        "defaultPeerId": PEER_ID,
        "defaultMetadataFile": METADATA_FILE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return Response(content=body, media_type="application/javascript")


@app.get("/metadata-stream")
async def metadata_stream() -> StreamingResponse:
    # Backward-compatible single-file stream.
    return StreamingResponse(_metadata_generator(Path(METADATA_FILE)), media_type="text/event-stream")


@app.post("/api/runs")
async def start_run(req: StartRunRequest) -> RunInfo:
    run_id = uuid.uuid4().hex
    peer_id = f"stream-{run_id[:10]}"
    metadata_file = f"/tmp/results-{run_id[:10]}.jsonl"

    start_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/user_defined_pipelines/{PIPELINE_NAME}"
    payload = {
        "source": {"uri": req.rtspUrl, "type": "uri"},
        "destination": {
            "metadata": {"type": "file", "path": metadata_file, "format": "json-lines"},
            "frame": {"type": "webrtc", "peer-id": peer_id, "bitrate": 5000},
        },
        "parameters": {
            "captioner-prompt": (req.prompt or "").strip() or "Describe what you see in the image in one sentence.",
            "captioner_model_name": (req.modelName or "").strip() or "OpenGVLab/InternVL2-2B",
            "captioner_max_new_tokens": req.maxNewTokens,
        },
    }

    raw = _http_json("POST", start_url, payload=payload)
    pipeline_id = raw.replace('"', "").strip()
    if not pipeline_id:
        raise HTTPException(status_code=502, detail={"message": "Pipeline server returned empty pipeline id", "body": raw})

    info = RunInfo(runId=run_id[:10], pipelineId=pipeline_id, peerId=peer_id, metadataFile=metadata_file)
    RUNS[info.runId] = info
    return info


@app.get("/api/runs")
async def list_runs() -> list[RunInfo]:
    return list(RUNS.values())


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
    return {"status": "stopped", "runId": run_id}


@app.get("/api/runs/{run_id}/metadata-stream")
async def run_metadata_stream(run_id: str) -> StreamingResponse:
    info = RUNS.get(run_id)
    if not info:
        raise HTTPException(status_code=404, detail={"message": "Run not found"})
    return StreamingResponse(_metadata_generator(Path(info.metadataFile)), media_type="text/event-stream")


@app.get("/system-stats")
async def system_stats() -> StreamingResponse:
    return StreamingResponse(_system_stats_generator(), media_type="text/event-stream")


@app.get("/")
async def root() -> FileResponse:
    return FileResponse(PUBLIC_DIR / "index.html")


app.mount("/", StaticFiles(directory=PUBLIC_DIR, html=True), name="public")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("app:app", host="0.0.0.0", port=APP_PORT, reload=True)
