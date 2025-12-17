"""
Live Video Captioning Dashboard - aiohttp Server

This module provides an async HTTP server using aiohttp for the live video
captioning dashboard. It exposes REST APIs for managing video processing
pipelines, streaming metadata, and monitoring system/GPU metrics.

API Documentation is available at /api/docs (Swagger UI).
"""

import asyncio
import json
import logging
import os
import threading
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, AsyncGenerator, Optional

import aiohttp
from aiohttp import web
from aiohttp.client_exceptions import ClientConnectionResetError
import aiohttp_cors
import psutil

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Swagger documentation support
from aiohttp_swagger3 import SwaggerDocs, SwaggerInfo, SwaggerUiSettings

APP_PORT = int(os.environ.get("DASHBOARD_PORT", "4173"))
METADATA_FILE = os.environ.get("METADATA_FILE", "/tmp/results.jsonl")
PEER_ID = os.environ.get("WEBRTC_PEER_ID", "genai_pipeline")
SIGNALING_URL = os.environ.get("SIGNALING_URL", "http://localhost:8889")
POLL_INTERVAL = float(os.environ.get("METADATA_POLL_SECONDS", "1"))

PIPELINE_SERVER_URL = os.environ.get("PIPELINE_SERVER_URL", "http://video-ingestion:8080")
PIPELINE_NAME = os.environ.get("PIPELINE_NAME", "genai_pipeline")

# GPU metrics configuration
# Qmassa runs on the host with sudo to see all GPU clients across containers
# The dashboard reads from the host-generated JSON file mounted at /tmp
QMASSA_ENABLED = os.environ.get("QMASSA_ENABLED", "true").lower() in ("true", "1", "yes")
QMASSA_JSON_FILE = Path(os.environ.get("QMASSA_JSON_FILE", "/tmp/qmassa-stats.json"))
QMASSA_POLL_INTERVAL_MS = int(os.environ.get("QMASSA_POLL_INTERVAL_MS", "1000"))

BASE_DIR = Path(__file__).parent
MODELS_DIR = Path(os.environ.get("MODELS_DIR", str(BASE_DIR / "ov_models")))
PUBLIC_DIR = BASE_DIR / "public"


@dataclass
class StartRunRequest:
    """Request payload for starting a new video processing run.
    
    Attributes:
        rtspUrl: The RTSP URL of the video source (required).
        prompt: The prompt to use for video captioning.
        modelName: The name of the model to use for captioning.
        maxNewTokens: Maximum number of new tokens to generate (1-4096).
        pipelineName: Optional pipeline name override.
    """
    rtspUrl: str
    prompt: str = "Describe what you see in the image in one sentence."
    modelName: str = "OpenGVLab/InternVL2-2B"
    maxNewTokens: int = 70
    pipelineName: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StartRunRequest":
        """Create a StartRunRequest from a dictionary.
        
        Args:
            data: Dictionary containing request parameters.
            
        Returns:
            StartRunRequest instance.
            
        Raises:
            ValueError: If required fields are missing or invalid.
        """
        if "rtspUrl" not in data or not data["rtspUrl"]:
            raise ValueError("rtspUrl is required and must not be empty")
        
        max_tokens = data.get("maxNewTokens", 70)
        if not isinstance(max_tokens, int) or max_tokens < 1 or max_tokens > 4096:
            raise ValueError("maxNewTokens must be an integer between 1 and 4096")
        
        return cls(
            rtspUrl=data["rtspUrl"],
            prompt=data.get("prompt", cls.prompt),
            modelName=data.get("modelName", cls.modelName),
            maxNewTokens=max_tokens,
            pipelineName=data.get("pipelineName"),
        )


@dataclass
class RunInfo:
    """Information about an active video processing run.
    
    Attributes:
        runId: Unique identifier for this run.
        pipelineId: Pipeline server's identifier for the pipeline instance.
        peerId: WebRTC peer ID for video streaming.
        metadataFile: Path to the metadata output file.
    """
    runId: str
    pipelineId: str
    peerId: str
    metadataFile: str

    def to_dict(self) -> dict[str, str]:
        """Convert to dictionary for JSON serialization."""
        return {
            "runId": self.runId,
            "pipelineId": self.pipelineId,
            "peerId": self.peerId,
            "metadataFile": self.metadataFile,
        }


# Active runs storage
RUNS: dict[str, RunInfo] = {}


@dataclass
class GPUMetrics:
    """GPU metrics collected from qmassa."""
    available: bool = False
    device_name: str = ""
    driver: str = ""
    pci_slot: str = ""
    device_type: str = ""  # Integrated, Discrete, Unknown
    # Engine utilization (0-100 percent)
    engines: dict[str, float] = field(default_factory=dict)
    total_engine_percent: float = 0.0
    # Memory stats
    smem_used_bytes: int = 0
    smem_total_bytes: int = 0
    vram_used_bytes: int = 0
    vram_total_bytes: int = 0
    # Frequency (MHz)
    freq_actual: float = 0.0
    freq_max: float = 0.0
    # Power (Watts)
    power_gpu: float = 0.0
    power_package: float = 0.0
    # Temperature (Celsius)
    temperature: float = 0.0
    # Error message if any
    error: Optional[str] = None


class QmassaCollector:
    """Collector for GPU metrics from host-generated qmassa JSON file.
    
    Qmassa must be run on the host with sudo to see all GPU clients:
        sudo qmassa -x -t /tmp/qmassa-stats.json -m 1000
    
    The dashboard container reads from this file via the /tmp volume mount.
    """

    def __init__(self, json_file: Path, poll_interval_ms: int = 1000):
        self.json_file = json_file
        self.poll_interval_ms = poll_interval_ms
        self._lock = threading.Lock()
        self._latest_metrics: GPUMetrics = GPUMetrics()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the background reader thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_reader, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background reader thread."""
        self._running = False

    def get_metrics(self) -> GPUMetrics:
        """Get the latest GPU metrics."""
        with self._lock:
            return self._latest_metrics

    def _run_reader(self) -> None:
        """Periodically read and parse the qmassa JSON file."""
        import time
        wait_logged = False
        
        while self._running:
            try:
                if not self.json_file.exists():
                    if not wait_logged:
                        with self._lock:
                            self._latest_metrics = GPUMetrics(
                                error="Run on host: sudo qmassa -x -t /tmp/qmassa-stats.json "
                            )
                        wait_logged = True
                else:
                    metrics = self._parse_json_file()
                    with self._lock:
                        self._latest_metrics = metrics
                    wait_logged = False
            except Exception as e:
                with self._lock:
                    self._latest_metrics = GPUMetrics(error=f"Parse error: {e}")

            time.sleep(self.poll_interval_ms / 1000.0)

    def _parse_json_file(self) -> GPUMetrics:
        """Parse the qmassa JSON output file."""
        if not self.json_file.exists():
            return GPUMetrics(error="Waiting for qmassa data...")

        try:
            content = self.json_file.read_text(encoding="utf-8")
            # qmassa writes JSON lines format
            lines = [line.strip() for line in content.strip().split("\n") if line.strip()]
            if not lines:
                return GPUMetrics(error="No data yet")

            # Parse the last line (most recent data)
            data = json.loads(lines[-1])
            return self._extract_metrics(data)

        except json.JSONDecodeError as e:
            return GPUMetrics(error=f"JSON parse error: {e}")
        except Exception as e:
            return GPUMetrics(error=f"Read error: {e}")

    def _extract_metrics(self, data: dict) -> GPUMetrics:
        """Extract GPU metrics from qmassa JSON data."""
        metrics = GPUMetrics(available=True)

        # qmassa v1.2.0 format: devs_state array contains GPU devices
        devices = data.get("devs_state", [])
        if not devices:
            # Try alternate formats
            devices = data.get("devices", [])
            if not devices and isinstance(data, list):
                devices = data

        if not devices:
            return GPUMetrics(error="No GPU devices found in qmassa output")

        # Filter out non-GPU devices (like ASPEED BMC) and prefer Intel/AMD discrete GPUs
        gpu_devices = []
        for dev in devices:
            drv_name = dev.get("drv_name", "")
            dev_type = dev.get("dev_type", "Unknown")
            # Skip ASPEED BMC and unknown devices without engine support
            if drv_name in ("ast", ""):
                continue
            # Prefer discrete GPUs (xe, i915, amdgpu drivers)
            if drv_name in ("xe", "i915", "amdgpu") or dev_type in ("Discrete", "Integrated"):
                gpu_devices.append(dev)

        if not gpu_devices:
            # Fallback to any device with stats
            gpu_devices = [d for d in devices if d.get("dev_stats", {}).get("eng_usage")]

        if not gpu_devices:
            return GPUMetrics(error="No supported GPU devices found")

        # Use the first GPU device (or could aggregate multiple)
        dev = gpu_devices[0]

        # Basic device info
        metrics.pci_slot = dev.get("pci_dev", "")
        metrics.device_name = dev.get("vdr_dev", dev.get("pci_id", "Unknown GPU"))
        metrics.driver = dev.get("drv_name", "")
        metrics.device_type = dev.get("dev_type", "Unknown")

        # Get dev_stats which contains all the metrics
        dev_stats = dev.get("dev_stats", {})

        # Engine utilization - eng_usage is a dict of engine_name -> [usage_history]
        # First, get device-level engine usage
        eng_usage = dev_stats.get("eng_usage", {})
        if isinstance(eng_usage, dict):
            for name, usage_list in eng_usage.items():
                if isinstance(usage_list, list) and len(usage_list) > 0:
                    # Get the latest value (last in the array)
                    latest = usage_list[-1] if usage_list else 0
                    metrics.engines[name] = float(latest) if latest else 0.0
                elif isinstance(usage_list, (int, float)):
                    metrics.engines[name] = float(usage_list)

        # If device-level engine usage is all zeros, aggregate from client stats
        # This is needed because qmassa reports per-client engine usage
        if all(v == 0.0 for v in metrics.engines.values()):
            clis_stats = dev.get("clis_stats", [])
            aggregated_engines: dict[str, float] = {}
            for cli in clis_stats:
                cli_eng = cli.get("eng_usage", {})
                for name, usage_list in cli_eng.items():
                    if isinstance(usage_list, list) and len(usage_list) > 0:
                        latest = usage_list[-1] if usage_list else 0
                        if latest:
                            aggregated_engines[name] = aggregated_engines.get(name, 0) + float(latest)
                    elif isinstance(usage_list, (int, float)) and usage_list:
                        aggregated_engines[name] = aggregated_engines.get(name, 0) + float(usage_list)
            
            # Use aggregated client engine usage if available
            if aggregated_engines:
                metrics.engines = aggregated_engines

        # Calculate total engine usage (average of all engines)
        if metrics.engines:
            metrics.total_engine_percent = sum(metrics.engines.values()) / len(metrics.engines)

        # Memory stats - mem_info is an array of snapshots
        mem_info_list = dev_stats.get("mem_info", [])
        if mem_info_list and isinstance(mem_info_list, list):
            # Get the latest memory snapshot
            mem = mem_info_list[-1] if mem_info_list else {}
            if isinstance(mem, dict):
                metrics.smem_used_bytes = int(mem.get("smem_used", 0) or 0)
                metrics.smem_total_bytes = int(mem.get("smem_total", 0) or 0)
                metrics.vram_used_bytes = int(mem.get("vram_used", 0) or 0)
                metrics.vram_total_bytes = int(mem.get("vram_total", 0) or 0)

        # Frequency - freqs is an array of GT frequency snapshots
        freqs_list = dev_stats.get("freqs", [])
        freq_limits = dev.get("freq_limits", [])
        if freqs_list and isinstance(freqs_list, list) and len(freqs_list) > 0:
            # Get the latest frequency snapshot (last in outer array)
            latest_freqs = freqs_list[-1] if freqs_list else []
            if latest_freqs and isinstance(latest_freqs, list) and len(latest_freqs) > 0:
                # First GT (gt0) frequencies
                gt0_freq = latest_freqs[0] if latest_freqs else {}
                if isinstance(gt0_freq, dict):
                    metrics.freq_actual = float(gt0_freq.get("act_freq", 0) or 0)
                    metrics.freq_max = float(gt0_freq.get("max_freq", 0) or 0)

        # Also check freq_limits for max frequency
        if freq_limits and isinstance(freq_limits, list) and len(freq_limits) > 0:
            gt0_limits = freq_limits[0] if freq_limits else {}
            if isinstance(gt0_limits, dict) and metrics.freq_max == 0:
                metrics.freq_max = float(gt0_limits.get("maximum", 0) or 0)

        # Power - power is an array of power snapshots
        power_list = dev_stats.get("power", [])
        if power_list and isinstance(power_list, list) and len(power_list) > 0:
            # Get the latest power snapshot
            power = power_list[-1] if power_list else {}
            if isinstance(power, dict):
                metrics.power_gpu = float(power.get("gpu_cur_power", 0) or 0)
                metrics.power_package = float(power.get("pkg_cur_power", 0) or 0)

        # Temperature - temps is an array
        temps_list = dev_stats.get("temps", [])
        if temps_list and isinstance(temps_list, list) and len(temps_list) > 0:
            # Get latest temperature
            temp = temps_list[-1] if temps_list else 0
            if isinstance(temp, (int, float)):
                metrics.temperature = float(temp)
            elif isinstance(temp, dict):
                metrics.temperature = float(temp.get("value", temp.get("current", 0)) or 0)

        return metrics


# Global GPU metrics collector
_gpu_collector: Optional[QmassaCollector] = None


def get_gpu_collector() -> Optional[QmassaCollector]:
    """Get or create the GPU metrics collector."""
    global _gpu_collector
    if not QMASSA_ENABLED:
        return None
    if _gpu_collector is None:
        _gpu_collector = QmassaCollector(QMASSA_JSON_FILE, QMASSA_POLL_INTERVAL_MS)
        _gpu_collector.start()
    return _gpu_collector


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


async def _discover_pipelines_remote() -> list[str]:
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines"
    try:
        raw = await _http_json_async("GET", url, timeout=10)
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
                    elif isinstance(item, dict) and isinstance(item.get("version"), str):
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


async def _metadata_generator(path: Path) -> AsyncGenerator[str, None]:
    last_payload = None
    while True:
        latest = _read_latest_line(path)
        if latest and latest != last_payload:
            last_payload = latest
            yield f"data: {latest}\n\n"
        await asyncio.sleep(POLL_INTERVAL)


# Global aiohttp session for connection pooling
_http_session: Optional[aiohttp.ClientSession] = None


async def get_http_session() -> aiohttp.ClientSession:
    """Get or create the global aiohttp session.
    
    Uses a TCPConnector with increased limits to support many concurrent
    pipeline connections. Default limits are increased to handle multiple
    simultaneous streams.
    """
    global _http_session
    if _http_session is None or _http_session.closed:
        timeout = aiohttp.ClientTimeout(total=120, connect=10)
        # Increase connection limits for handling many concurrent pipelines
        connector = aiohttp.TCPConnector(
            limit=200,           # Total connection limit
            limit_per_host=50,   # Connections per host
            ttl_dns_cache=300,   # DNS cache TTL
            enable_cleanup_closed=True,
        )
        _http_session = aiohttp.ClientSession(timeout=timeout, connector=connector)
    return _http_session


class HTTPException(Exception):
    """HTTP exception for error handling in aiohttp routes.
    
    Attributes:
        status_code: The HTTP status code.
        detail: Error details (string or dict).
    """
    def __init__(self, status_code: int, detail: Any = None):
        self.status_code = status_code
        self.detail = detail
        super().__init__(str(detail))


async def _http_json_async(
    method: str,
    url: str,
    payload: Optional[dict[str, Any]] = None,
    timeout: Optional[int] = None
) -> str:
    """Perform an async HTTP request using aiohttp.
    
    Args:
        method: HTTP method (GET, POST, DELETE, etc.).
        url: Target URL.
        payload: Optional JSON payload for POST/PUT requests.
        timeout: Optional request timeout in seconds.
        
    Returns:
        Response body as string.
        
    Raises:
        HTTPException: On HTTP errors, unreachable server, or timeout.
    """
    session = await get_http_session()
    headers = {"Accept": "application/json"}
    
    kwargs: dict[str, Any] = {"headers": headers}
    if payload is not None:
        kwargs["json"] = payload
        headers["Content-Type"] = "application/json"
    
    if timeout is not None:
        kwargs["timeout"] = aiohttp.ClientTimeout(total=timeout)
    
    try:
        async with session.request(method, url, **kwargs) as resp:
            body = await resp.text()
            if resp.status >= 400:
                raise HTTPException(
                    status_code=502,
                    detail={
                        "message": "Pipeline server error",
                        "status": resp.status,
                        "body": body
                    }
                )
            return body
    except aiohttp.ClientError as err:
        raise HTTPException(
            status_code=502,
            detail={"message": "Pipeline server unreachable", "error": str(err)}
        )
    except asyncio.TimeoutError:
        raise HTTPException(
            status_code=504,
            detail={"message": "Pipeline server timeout"}
        )


async def _system_stats_generator() -> AsyncGenerator[str, None]:
    """Generate system statistics as Server-Sent Events.
    
    Yields CPU, memory, and GPU statistics periodically.
    
    Yields:
        SSE-formatted strings with JSON stats data.
    """
    while True:
        cpu = await asyncio.to_thread(psutil.cpu_percent, interval=0.5)
        mem = psutil.virtual_memory()

        # Get GPU stats from qmassa collector
        gpu_stats = _get_gpu_stats()

        stats = {
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
        yield f"data: {json.dumps(stats)}\n\n"
        await asyncio.sleep(POLL_INTERVAL)


def _get_gpu_stats() -> dict[str, Any]:
    """Get GPU stats from qmassa collector."""
    collector = get_gpu_collector()
    if not collector:
        return {"available": False, "error": "GPU monitoring disabled"}

    metrics = collector.get_metrics()

    if metrics.error:
        return {"available": False, "error": metrics.error}

    if not metrics.available:
        return {"available": False, "error": "No GPU data available"}

    result: dict[str, Any] = {
        "available": True,
        "device_name": metrics.device_name,
        "driver": metrics.driver,
        "device_type": metrics.device_type,
        "usage_percent": round(metrics.total_engine_percent, 1),
        "engines": {k: round(v, 1) for k, v in metrics.engines.items()},
    }

    # VRAM (device memory - for discrete GPUs)
    if metrics.vram_total_bytes > 0:
        result["vram_used_gb"] = round(metrics.vram_used_bytes / (1024**3), 2)
        result["vram_total_gb"] = round(metrics.vram_total_bytes / (1024**3), 2)
        result["vram_percent"] = round(
            (metrics.vram_used_bytes / metrics.vram_total_bytes) * 100, 1
        ) if metrics.vram_total_bytes else None

    # System memory used by GPU
    if metrics.smem_total_bytes > 0:
        result["smem_used_gb"] = round(metrics.smem_used_bytes / (1024**3), 2)
        result["smem_total_gb"] = round(metrics.smem_total_bytes / (1024**3), 2)

    # Frequency
    if metrics.freq_actual > 0:
        result["freq_actual"] = round(metrics.freq_actual)
    if metrics.freq_max > 0:
        result["freq_max"] = round(metrics.freq_max)

    # Power
    if metrics.power_gpu > 0:
        result["power_gpu"] = round(metrics.power_gpu, 1)
    if metrics.power_package > 0:
        result["power_package"] = round(metrics.power_package, 1)

    # Temperature
    if metrics.temperature > 0:
        result["temperature"] = round(metrics.temperature, 1)

    return result


# ============================================================================
# aiohttp Route Handlers
# ============================================================================

async def runtime_config(request: web.Request) -> web.Response:
    """
    ---
    summary: Get runtime configuration
    description: Returns JavaScript configuration for the frontend client.
    tags:
      - Configuration
    responses:
      "200":
        description: JavaScript runtime configuration
        content:
          application/javascript:
            schema:
              type: string
    """
    payload = {
        "signalingUrl": SIGNALING_URL,
        "defaultPeerId": PEER_ID,
        "defaultMetadataFile": METADATA_FILE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return web.Response(text=body, content_type="application/javascript")


async def list_models(request: web.Request) -> web.Response:
    """
    ---
    summary: List available models
    description: Returns a list of available captioning models from the models directory.
    tags:
      - Models
    responses:
      "200":
        description: List of available models
        content:
          application/json:
            schema:
              type: object
              properties:
                models:
                  type: array
                  items:
                    type: string
                  description: List of model names
    """
    models = _discover_models(MODELS_DIR)
    return web.json_response({"models": models})


async def list_pipelines(request: web.Request) -> web.Response:
    """
    ---
    summary: List available pipelines
    description: Returns a list of available video processing pipelines from the pipeline server.
    tags:
      - Pipelines
    responses:
      "200":
        description: List of available pipelines
        content:
          application/json:
            schema:
              type: object
              properties:
                pipelines:
                  type: array
                  items:
                    type: string
                  description: List of pipeline names
    """
    names = await _discover_pipelines_remote()
    return web.json_response({"pipelines": names})


async def metadata_stream(request: web.Request) -> web.StreamResponse:
    """
    ---
    summary: Stream metadata (SSE)
    description: Server-Sent Events stream of video captioning metadata from the default metadata file.
    tags:
      - Streaming
    responses:
      "200":
        description: SSE stream of metadata
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
        async for chunk in _metadata_generator(Path(METADATA_FILE)):
            await response.write(chunk.encode("utf-8"))
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        pass
    
    return response


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
    try:
        data = await request.json()
        req = StartRunRequest.from_dict(data)
    except json.JSONDecodeError:
        logger.error("start_run: Invalid JSON in request body")
        return web.json_response({"error": "Invalid JSON"}, status=400)
    except ValueError as e:
        logger.error(f"start_run: Validation error - {e}")
        return web.json_response({"error": str(e)}, status=400)
    
    run_id = uuid.uuid4().hex
    peer_id = f"stream-{run_id[:10]}"
    metadata_file = f"/tmp/results-{run_id[:10]}.jsonl"

    pipeline_name = (req.pipelineName or PIPELINE_NAME).strip() or PIPELINE_NAME
    
    logger.info(f"start_run: Starting pipeline '{pipeline_name}' with run_id={run_id[:10]}, "
                f"rtspUrl={req.rtspUrl}, model={req.modelName}")

    start_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/user_defined_pipelines/{pipeline_name}"
    payload = {
        "source": {"uri": req.rtspUrl, "type": "uri"},
        "destination": {
            "metadata": {"type": "file", "path": metadata_file, "format": "json-lines"},
            "frame": {"type": "webrtc", "peer-id": peer_id, "bitrate": 3000},
        },
        "parameters": {
            "captioner-prompt": (req.prompt or "").strip() or "Describe what you see in the image in one sentence.",
            "captioner_model_name": (req.modelName or "").strip() or "OpenGVLab/InternVL2-2B",
            "captioner_max_new_tokens": req.maxNewTokens,
        },
    }

    try:
        logger.info(f"start_run: Sending request to pipeline server at {start_url}")
        raw = await _http_json_async("POST", start_url, payload=payload, timeout=120)
        logger.info(f"start_run: Pipeline server response: {raw[:100] if raw else 'empty'}")
    except HTTPException as e:
        logger.error(f"start_run: Pipeline server error - status={e.status_code}, detail={e.detail}")
        return web.json_response(e.detail, status=e.status_code)
    
    pipeline_id = raw.replace('"', "").strip()
    if not pipeline_id:
        logger.error(f"start_run: Pipeline server returned empty pipeline id, body: {raw}")
        return web.json_response(
            {"message": "Pipeline server returned empty pipeline id", "body": raw},
            status=502
        )

    info = RunInfo(runId=run_id[:10], pipelineId=pipeline_id, peerId=peer_id, metadataFile=metadata_file)
    RUNS[info.runId] = info
    logger.info(f"start_run: Successfully started run {info.runId} with pipeline_id={pipeline_id}, "
                f"total active runs: {len(RUNS)}")
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
    return web.json_response([info.to_dict() for info in RUNS.values()])


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
    info = RUNS.get(run_id)
    if not info:
        return web.json_response({"message": "Run not found"}, status=404)
    return web.json_response(info.to_dict())


async def stop_run(request: web.Request) -> web.Response:
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
    info = RUNS.get(run_id)
    if not info:
        return web.json_response({"message": "Run not found"}, status=404)
    
    stop_url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines/{info.pipelineId}"
    try:
        await _http_json_async("DELETE", stop_url, timeout=15)
    except HTTPException as e:
        # If pipeline server says 404, pipeline may already be stopped - clean up anyway
        if e.status_code == 502 and e.detail and e.detail.get("status") == 404:
            pass  # Pipeline already gone, proceed with cleanup
        else:
            # Remove from local tracking even on error to allow UI cleanup
            RUNS.pop(run_id, None)
            return web.json_response(e.detail, status=e.status_code)
    
    RUNS.pop(run_id, None)
    try:
        Path(info.metadataFile).unlink(missing_ok=True)
    except OSError:
        pass
    return web.json_response({"status": "stopped", "runId": run_id})


async def run_metadata_stream(request: web.Request) -> web.StreamResponse:
    """
    ---
    summary: Stream run metadata (SSE)
    description: Server-Sent Events stream of metadata for a specific run.
    tags:
      - Runs
      - Streaming
    parameters:
      - name: run_id
        in: path
        required: true
        schema:
          type: string
        description: The run ID
    responses:
      "200":
        description: SSE stream of run metadata
        content:
          text/event-stream:
            schema:
              type: string
      "404":
        description: Run not found
    """
    run_id = request.match_info["run_id"]
    info = RUNS.get(run_id)
    if not info:
        return web.json_response({"message": "Run not found"}, status=404)
    
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
        async for chunk in _metadata_generator(Path(info.metadataFile)):
            await response.write(chunk.encode("utf-8"))
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        pass
    
    return response


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
            # Get current runs snapshot
            current_runs = dict(RUNS)
            
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
                    latest = _read_latest_line(Path(info.metadataFile))
                    if latest and latest != last_payloads.get(run_id):
                        last_payloads[run_id] = latest
                        # Parse and wrap with runId
                        try:
                            data = json.loads(latest)
                        except json.JSONDecodeError:
                            data = latest
                        event = {"runId": run_id, "data": data}
                        await response.write(f"data: {json.dumps(event)}\n\n".encode("utf-8"))
                except Exception:
                    # Skip this run if there's an error reading its metadata
                    pass
            
            await asyncio.sleep(POLL_INTERVAL)
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        pass
    
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
        async for chunk in _system_stats_generator():
            await response.write(chunk.encode("utf-8"))
    except (asyncio.CancelledError, ClientConnectionResetError, ConnectionResetError):
        # Client disconnected, gracefully stop streaming
        pass
    
    return response


async def gpu_stats(request: web.Request) -> web.Response:
    """
    ---
    summary: Get current GPU stats
    description: Returns current GPU statistics as a JSON response.
    tags:
      - Monitoring
    responses:
      "200":
        description: Current GPU statistics
        content:
          application/json:
            schema:
              type: object
              properties:
                available:
                  type: boolean
                  description: Whether GPU monitoring is available
                device_name:
                  type: string
                  description: GPU device name
                driver:
                  type: string
                  description: GPU driver name
                device_type:
                  type: string
                  description: "Device type (Integrated, Discrete, Unknown)"
                usage_percent:
                  type: number
                  description: GPU usage percentage
                engines:
                  type: object
                  description: Engine utilization breakdown
                vram_used_gb:
                  type: number
                  description: VRAM used in GB
                vram_total_gb:
                  type: number
                  description: Total VRAM in GB
                vram_percent:
                  type: number
                  description: VRAM usage percentage
                freq_actual:
                  type: number
                  description: Current GPU frequency in MHz
                freq_max:
                  type: number
                  description: Maximum GPU frequency in MHz
                power_gpu:
                  type: number
                  description: GPU power consumption in Watts
                temperature:
                  type: number
                  description: GPU temperature in Celsius
                error:
                  type: string
                  description: Error message if monitoring failed
    """
    return web.json_response(_get_gpu_stats())


async def index_handler(request: web.Request) -> web.FileResponse:
    """
    ---
    summary: Serve the main dashboard page
    description: Returns the main HTML page for the dashboard.
    tags:
      - Static
    responses:
      "200":
        description: HTML page
        content:
          text/html:
            schema:
              type: string
    """
    return web.FileResponse(PUBLIC_DIR / "index.html")


# ============================================================================
# Application Lifecycle
# ============================================================================

async def on_startup(app: web.Application) -> None:
    """Initialize resources on application startup.
    
    - Starts the GPU metrics collector if enabled.
    """
    if QMASSA_ENABLED:
        get_gpu_collector()


async def on_cleanup(app: web.Application) -> None:
    """Cleanup resources on application shutdown.
    
    - Stops the GPU metrics collector.
    - Closes the HTTP client session.
    """
    global _gpu_collector, _http_session
    if _gpu_collector:
        _gpu_collector.stop()
        _gpu_collector = None
    if _http_session and not _http_session.closed:
        await _http_session.close()
        _http_session = None


def create_app() -> web.Application:
    """Create and configure the aiohttp web application.
    
    Returns:
        Configured aiohttp Application instance with all routes and middleware.
    """
    app = web.Application()
    
    # Setup Swagger documentation
    swagger = SwaggerDocs(
        app,
        swagger_ui_settings=SwaggerUiSettings(path="/api/docs"),
        info=SwaggerInfo(
            title="Live Video Captioning Dashboard API",
            version="1.0.0",
            description="""
REST API for the Live Video Captioning Dashboard.

## Features

- **Runs Management**: Create, list, and stop video processing runs
- **Models**: List available captioning models
- **Pipelines**: List available video processing pipelines
- **Monitoring**: Real-time CPU, memory, and GPU statistics
- **Streaming**: Server-Sent Events for metadata and stats

## Authentication

Currently no authentication is required.

## Rate Limiting

No rate limiting is applied.
            """,
        ),
    )
    
    # Setup CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*",
        )
    })
    
    # Register API routes with Swagger documentation
    swagger.add_routes([
        web.get("/runtime-config.js", runtime_config),
        web.get("/api/models", list_models),
        web.get("/api/pipelines", list_pipelines),
        web.get("/metadata-stream", metadata_stream),
        web.post("/api/runs", start_run),
        web.get("/api/runs", list_runs),
        web.get("/api/runs/{run_id}", get_run),
        web.delete("/api/runs/{run_id}", stop_run),
        web.get("/api/runs/{run_id}/metadata-stream", run_metadata_stream),
        web.get("/api/runs/metadata-stream", all_runs_metadata_stream),
        web.get("/system-stats", system_stats),
        web.get("/api/gpu-stats", gpu_stats),
    ])
    
    # Add root handler and static files (outside swagger for proper static file handling)
    app.router.add_get("/", index_handler)
    app.router.add_static("/", PUBLIC_DIR, name="static", show_index=False)
    
    # Apply CORS to all routes
    for route in list(app.router.routes()):
        try:
            cors.add(route)
        except ValueError:
            # Some routes may not support CORS
            pass
    
    # Register lifecycle handlers
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    
    return app


# Create the application instance
app = create_app()


if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=APP_PORT)
