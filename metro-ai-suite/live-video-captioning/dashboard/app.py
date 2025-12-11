import asyncio
import json
import os
from pathlib import Path
from typing import AsyncGenerator, Optional

import psutil
from fastapi import FastAPI, Response
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

APP_PORT = int(os.environ.get("DASHBOARD_PORT", "4173"))
METADATA_FILE = os.environ.get("METADATA_FILE", "/tmp/results.jsonl")
PEER_ID = os.environ.get("WEBRTC_PEER_ID", "genai_pipeline")
SIGNALING_URL = os.environ.get("SIGNALING_URL", "http://localhost:8889")
POLL_INTERVAL = float(os.environ.get("METADATA_POLL_SECONDS", "1"))

BASE_DIR = Path(__file__).parent
PUBLIC_DIR = BASE_DIR / "public"

app = FastAPI()


def _read_latest_line() -> Optional[str]:
    path = Path(METADATA_FILE)
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


async def _metadata_generator() -> AsyncGenerator[str, None]:
    last_payload = None
    while True:
        latest = _read_latest_line()
        if latest and latest != last_payload:
            last_payload = latest
            yield f"data: {latest}\n\n"
        await asyncio.sleep(POLL_INTERVAL)


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
        "peerId": PEER_ID,
        "signalingUrl": SIGNALING_URL,
        "metadataFile": METADATA_FILE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return Response(content=body, media_type="application/javascript")


@app.get("/metadata-stream")
async def metadata_stream() -> StreamingResponse:
    return StreamingResponse(_metadata_generator(), media_type="text/event-stream")


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
