#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import importlib
import json
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

try:
    import yaml
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: PyYAML. Install with: pip install pyyaml"
    ) from exc

try:
    ws_module = importlib.import_module("websockets.sync.client")
    ws_connect = getattr(ws_module, "connect", None)
except Exception:
    ws_connect = None


@dataclass
class RunInfo:
    run_id: str
    pipeline_id: str
    model_name: str


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def utc_now_compact() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9._-]+", "_", value.strip())
    cleaned = cleaned.strip("._-")
    return cleaned or "unknown"


def get_nested(data: Any, dotted_path: str, default: Any = None) -> Any:
    cur = data
    for part in dotted_path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def http_json(
    method: str,
    url: str,
    payload: dict[str, Any] | None = None,
    timeout_sec: int = 20,
) -> Any:
    body = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        body = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    req = Request(url=url, method=method, data=body, headers=headers)
    try:
        with urlopen(req, timeout=timeout_sec) as response:
            raw = response.read().decode("utf-8")
            if not raw:
                return None
            try:
                return json.loads(raw)
            except json.JSONDecodeError:
                return raw
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} for {method} {url}: {detail}") from exc
    except URLError as exc:
        raise RuntimeError(f"Network error for {method} {url}: {exc}") from exc


def start_run(
    api_base: str,
    runs_endpoint: str,
    payload: dict[str, Any],
    timeout_sec: int,
) -> RunInfo:
    result = http_json(
        "POST",
        f"{api_base.rstrip('/')}{runs_endpoint}",
        payload,
        timeout_sec=timeout_sec,
    )
    if not isinstance(result, dict):
        raise RuntimeError(f"Unexpected start run response: {result}")
    return RunInfo(
        run_id=str(result.get("runId", "")),
        pipeline_id=str(result.get("pipelineId", "")),
        model_name=str(result.get("modelName", payload.get("modelName", ""))),
    )


def stop_run(api_base: str, stop_template: str, run_id: str, timeout_sec: int) -> None:
    endpoint = stop_template.replace("{run_id}", run_id)
    try:
        http_json("DELETE", f"{api_base.rstrip('/')}{endpoint}", timeout_sec=timeout_sec)
    except Exception as exc:
        print(f"[WARN] Failed to stop run {run_id}: {exc}")


def list_active_runs(api_base: str, runs_endpoint: str, timeout_sec: int) -> list[dict[str, Any]]:
    data = http_json("GET", f"{api_base.rstrip('/')}{runs_endpoint}", timeout_sec=timeout_sec)
    if isinstance(data, list):
        return [item for item in data if isinstance(item, dict)]
    return []


def wait_for_no_active_runs(
    api_base: str,
    runs_endpoint: str,
    timeout_sec: int,
    poll_interval_sec: float = 1.0,
) -> None:
    start = time.time()
    while True:
        active_runs = list_active_runs(api_base, runs_endpoint, timeout_sec=timeout_sec)
        if not active_runs:
            return
        if (time.time() - start) >= timeout_sec:
            ids = [str(item.get("runId", "unknown")) for item in active_runs]
            raise RuntimeError(
                "Timed out waiting for active runs to stop before starting next run. "
                f"Still active: {ids}"
            )
        time.sleep(poll_interval_sec)


def wait_for_run_stopped(
    api_base: str,
    runs_endpoint: str,
    run_id: str,
    timeout_sec: int,
    poll_interval_sec: float = 1.0,
) -> None:
    start = time.time()
    while True:
        active_runs = list_active_runs(api_base, runs_endpoint, timeout_sec=timeout_sec)
        is_active = any(str(item.get("runId", "")) == run_id for item in active_runs)
        if not is_active:
            return
        if (time.time() - start) >= timeout_sec:
            raise RuntimeError(f"Timed out waiting for run {run_id} to stop")
        time.sleep(poll_interval_sec)


def video_name_from_rtsp(rtsp_url: str) -> str:
    parsed = urlparse(rtsp_url)
    path = (parsed.path or "").strip("/")
    if path:
        return safe_name(path.split("/")[-1])
    return safe_name(parsed.netloc or "video")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid YAML root object in {path}")
    return data


def save_yaml(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def read_rtsp_list(rtsp_file: Path, ignore_blank: bool, ignore_comments: bool) -> list[str]:
    lines = [line.rstrip("\n") for line in rtsp_file.read_text(encoding="utf-8").splitlines()]
    result: list[str] = []
    for line in lines:
        s = line.strip()
        if ignore_blank and not s:
            continue
        if ignore_comments and s.startswith("#"):
            continue
        result.append(s)
    return result


def discover_models(models_dir: Path, include_models: list[str], exclude_models: list[str]) -> list[str]:
    if include_models:
        models = include_models[:]
    else:
        models = [p.name for p in models_dir.iterdir() if p.is_dir()]
    excluded = set(exclude_models)
    models = [m for m in models if m not in excluded]
    models.sort()
    return models


def extract_rest_requests(config: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    pairs: list[tuple[str, dict[str, Any]]] = []
    for key, value in config.items():
        if key.startswith("rest_request_") and isinstance(value, dict):
            pairs.append((key, value))
    pairs.sort(key=lambda item: item[0])
    return pairs


class SystemMetricsCollector:
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._history: list[tuple[float, dict[str, float | None]]] = []

    def start(self) -> None:
        if ws_connect is None:
            print("[WARN] websockets package unavailable; system metrics collection disabled")
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def snapshot_for(self, ts: float) -> dict[str, float | None]:
        with self._lock:
            if not self._history:
                return {
                    "cpu_percent": None,
                    "ram_percent": None,
                    "gpu_percent": None,
                    "gpu_temp_c": None,
                    "gpu_power_w": None,
                    "pkg_power_w": None,
                }
            chosen = self._history[-1][1]
            for hist_ts, payload in reversed(self._history):
                if hist_ts <= ts:
                    chosen = payload
                    break
            return dict(chosen)

    def _run(self) -> None:
        try:
            with ws_connect(self.ws_url) as ws:
                while not self._stop.is_set():
                    try:
                        raw = ws.recv(timeout=1)
                    except TimeoutError:
                        continue
                    if raw is None:
                        continue
                    data = json.loads(raw)
                    metrics = data.get("metrics")
                    if not isinstance(metrics, list):
                        continue
                    snapshot = self._flatten_metrics(metrics)
                    with self._lock:
                        self._history.append((time.time(), snapshot))
                        if len(self._history) > 5000:
                            self._history = self._history[-3000:]
        except Exception as exc:
            print(f"[WARN] Metrics collector stopped: {exc}")

    @staticmethod
    def _flatten_metrics(metrics: list[dict[str, Any]]) -> dict[str, float | None]:
        cpu = None
        ram = None
        gpu_values: list[float] = []
        gpu_temp = None
        gpu_power = None
        pkg_power = None

        for item in metrics:
            name = item.get("name")
            fields = item.get("fields") or {}
            tags = item.get("tags") or {}

            if name == "cpu" and isinstance(fields.get("usage_user"), (int, float)):
                cpu = float(fields["usage_user"])
            elif name == "mem" and isinstance(fields.get("used_percent"), (int, float)):
                ram = float(fields["used_percent"])
            elif name == "gpu_engine_usage" and isinstance(fields.get("usage"), (int, float)):
                gpu_values.append(float(fields["usage"]))
            elif name == "temp" and isinstance(fields.get("temp"), (int, float)):
                sensor = str(tags.get("sensor", ""))
                if "package" in sensor:
                    gpu_temp = float(fields["temp"])
            elif name == "gpu_power" and isinstance(fields.get("value"), (int, float)):
                ptype = tags.get("type")
                if ptype == "gpu_cur_power":
                    gpu_power = float(fields["value"])
                elif ptype == "pkg_cur_power":
                    pkg_power = float(fields["value"])

        gpu = max(gpu_values) if gpu_values else None
        return {
            "cpu_percent": cpu,
            "ram_percent": ram,
            "gpu_percent": gpu,
            "gpu_temp_c": gpu_temp,
            "gpu_power_w": gpu_power,
            "pkg_power_w": pkg_power,
        }


class SSECollector:
    def __init__(
        self,
        sse_url: str,
        run_id: str,
        caption_path: str,
        metrics_path: str,
        idle_timeout_sec: int,
    ):
        self.sse_url = sse_url
        self.run_id = run_id
        self.caption_path = caption_path
        self.metrics_path = metrics_path
        self.idle_timeout_sec = idle_timeout_sec

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._events: list[dict[str, Any]] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def events(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._events)

    def _run(self) -> None:
        req = Request(self.sse_url, headers={"Accept": "text/event-stream"})
        try:
            with urlopen(req, timeout=self.idle_timeout_sec) as resp:
                buffer: list[str] = []
                while not self._stop.is_set():
                    raw_line = resp.readline()
                    if not raw_line:
                        continue
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\n")
                    if line.startswith(":"):
                        continue
                    if line.startswith("data:"):
                        buffer.append(line[5:].strip())
                    elif line == "":
                        if not buffer:
                            continue
                        joined = "\n".join(buffer)
                        buffer.clear()
                        self._handle_event(joined)
        except Exception as exc:
            print(f"[WARN] SSE collector stopped for run {self.run_id}: {exc}")

    def _handle_event(self, event_text: str) -> None:
        try:
            envelope = json.loads(event_text)
        except json.JSONDecodeError:
            return
        if envelope.get("runId") != self.run_id:
            return

        event_ts = float(envelope.get("received_at", time.time()))
        caption = get_nested(envelope, self.caption_path, default=None)
        metrics = get_nested(envelope, self.metrics_path, default={})
        if caption is None:
            return

        item = {
            "event_ts": event_ts,
            "caption": str(caption),
            "sse_metrics": metrics if isinstance(metrics, dict) else {},
            "envelope": envelope,
        }
        with self._lock:
            self._events.append(item)


def ensure_csv(path: Path, fieldnames: list[str]) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()


def append_rows(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    ensure_csv(path, fieldnames)
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        for row in rows:
            writer.writerow(row)


def resolve_path(raw_path: str, project_root: Path, config_dir: Path) -> Path:
    p = Path(raw_path)
    if p.is_absolute():
        return p
    from_config = (config_dir / p).resolve()
    if from_config.exists():
        return from_config
    return (project_root / p).resolve()


def build_payload(rtsp_url: str, model_name: str, request_cfg: dict[str, Any]) -> dict[str, Any]:
    payload = dict(request_cfg)
    payload["rtspUrl"] = rtsp_url
    payload["modelName"] = model_name
    return payload


def is_pipeline_unreachable_error(exc: Exception) -> bool:
    msg = str(exc)
    checks = [
        "Pipeline server unreachable",
        "Temporary failure in name resolution",
        "HTTP 502",
    ]
    return any(token in msg for token in checks)


def run_benchmark(config_path: Path) -> None:
    project_root = config_path.parent.parent.parent
    config_dir = config_path.parent
    config = load_yaml(config_path)

    service = config["service"]
    inputs = config["inputs"]
    execution = config["execution"]
    collection = config["collection"]
    outputs = config["outputs"]

    rest_profiles = extract_rest_requests(config)
    if not rest_profiles:
        raise RuntimeError("No rest_request_* blocks found in config")

    rtsp_file = resolve_path(inputs["rtsp_list_file"], project_root, config_dir)
    rtsp_urls = read_rtsp_list(
        rtsp_file,
        ignore_blank=bool(inputs.get("ignore_blank_lines", True)),
        ignore_comments=bool(inputs.get("ignore_comments", True)),
    )
    if not rtsp_urls:
        raise RuntimeError(f"No RTSP URLs found in {rtsp_file}")

    model_cfg = config.get("model_selection", {})
    models_dir = resolve_path(model_cfg.get("ov_models_dir", "./ov_models"), project_root, config_dir)
    model_names = discover_models(
        models_dir=models_dir,
        include_models=model_cfg.get("include_models", []) or [],
        exclude_models=model_cfg.get("exclude_models", []) or [],
    )
    if not model_names:
        raise RuntimeError(f"No models found in {models_dir}")

    api_base = service["api_base_url"]
    runs_endpoint = service["runs_endpoint"]
    stop_template = service["runs_stop_endpoint_template"]
    sse_url = f"{api_base.rstrip('/')}{service['sse_endpoint']}"
    ws_url = service["ws_metrics_url"]
    start_timeout_sec = int(execution.get("start_timeout_sec", 45))
    stop_timeout_sec = int(execution.get("stop_timeout_sec", 25))
    start_retries = int(execution.get("retry_on_start_failure", 0))
    stop_wait_timeout_sec = int(execution.get("stop_wait_timeout_sec", 60))
    stop_wait_poll_interval_sec = float(execution.get("stop_wait_poll_interval_sec", 1.0))

    output_root = resolve_path(outputs["root_dir"], project_root, config_dir)
    request_template = outputs["folder_structure"]["request_subfolder_name_template"]
    csv_name = outputs["files"]["combined_caption_metrics_csv"]
    settings_name = outputs["files"]["run_settings_yaml"]

    fieldnames = [
        "timestamp_utc",
        "video_name",
        "rtsp_url",
        "rest_request_key",
        "model_name",
        "run_id",
        "pipeline_id",
        "caption",
        "caption_event_ts",
        "lag",
        "prompt",
        "maxNewTokens",
        "frameRate",
        "chunkSize",
        "frameWidth",
        "frameHeight",
        "cpu_percent",
        "ram_percent",
        "gpu_percent",
        "gpu_temp_c",
        "gpu_power_w",
        "pkg_power_w",
        "sse_metrics_json",
    ]

    metrics_collector = SystemMetricsCollector(ws_url)
    if collection.get("capture_websocket_metrics", True):
        metrics_collector.start()

    total_runs = 0
    total_captions = 0
    skipped_runs = 0

    try:
        for rtsp_url in rtsp_urls:
            video_name = video_name_from_rtsp(rtsp_url)
            for req_key, req_cfg in rest_profiles:
                req_folder = request_template.format(rest_request_key=req_key)
                request_root = output_root / video_name / safe_name(req_folder)
                request_root.mkdir(parents=True, exist_ok=True)

                for model_name in model_names:
                    wait_for_no_active_runs(
                        api_base,
                        runs_endpoint,
                        timeout_sec=stop_wait_timeout_sec,
                        poll_interval_sec=stop_wait_poll_interval_sec,
                    )

                    payload = build_payload(rtsp_url, model_name, req_cfg)
                    run_info = None
                    for attempt in range(start_retries + 1):
                        try:
                            run_info = start_run(
                                api_base,
                                runs_endpoint,
                                payload,
                                timeout_sec=start_timeout_sec,
                            )
                            break
                        except Exception as exc:
                            is_last = attempt >= start_retries
                            print(
                                f"[WARN] Start run failed (attempt {attempt + 1}/{start_retries + 1}) for model={model_name}: {exc}"
                            )
                            if is_last:
                                if is_pipeline_unreachable_error(exc):
                                    print(
                                        "[WARN] Skipping run because pipeline backend is unreachable from the app backend."
                                    )
                                    skipped_runs += 1
                                    run_info = None
                                    break
                                raise
                            time.sleep(2)

                    if run_info is None:
                        continue

                    total_runs += 1
                    print(
                        f"[RUN] video={video_name} request={req_key} model={model_name} run_id={run_info.run_id}"
                    )

                    run_folder_name = (
                        f"run_{utc_now_compact()}_{safe_name(model_name)}_{safe_name(run_info.run_id)}"
                    )
                    run_folder = request_root / run_folder_name
                    run_folder.mkdir(parents=True, exist_ok=False)

                    csv_path = run_folder / csv_name
                    ensure_csv(csv_path, fieldnames)

                    sse_collector = SSECollector(
                        sse_url=sse_url,
                        run_id=run_info.run_id,
                        caption_path=collection.get("sse_caption_path", "data.result"),
                        metrics_path=collection.get("sse_metrics_path", "data.metrics"),
                        idle_timeout_sec=int(collection.get("sse_idle_timeout_sec", 30)),
                    )
                    if collection.get("capture_sse", True):
                        sse_collector.start()

                    duration = int(execution.get("run_duration_sec", 120))
                    time.sleep(max(1, duration))

                    sse_collector.stop()
                    stop_run(api_base, stop_template, run_info.run_id, timeout_sec=stop_timeout_sec)
                    wait_for_run_stopped(
                        api_base,
                        runs_endpoint,
                        run_id=run_info.run_id,
                        timeout_sec=stop_wait_timeout_sec,
                        poll_interval_sec=stop_wait_poll_interval_sec,
                    )
                    wait_for_no_active_runs(
                        api_base,
                        runs_endpoint,
                        timeout_sec=stop_wait_timeout_sec,
                        poll_interval_sec=stop_wait_poll_interval_sec,
                    )

                    events = sse_collector.events()
                    rows: list[dict[str, Any]] = []
                    previous_event_ts: float | None = None
                    for event in events:
                        event_ts = float(event["event_ts"])
                        snap = metrics_collector.snapshot_for(event_ts)
                        lag = None if previous_event_ts is None else round(event_ts - previous_event_ts, 3)
                        previous_event_ts = event_ts

                        row = {
                            "timestamp_utc": utc_now_iso(),
                            "video_name": video_name,
                            "rtsp_url": rtsp_url,
                            "rest_request_key": req_key,
                            "model_name": model_name,
                            "run_id": run_info.run_id,
                            "pipeline_id": run_info.pipeline_id,
                            "caption": event["caption"],
                            "caption_event_ts": event_ts,
                            "lag": lag,
                            "prompt": req_cfg.get("prompt", ""),
                            "maxNewTokens": req_cfg.get("maxNewTokens"),
                            "frameRate": req_cfg.get("frameRate"),
                            "chunkSize": req_cfg.get("chunkSize"),
                            "frameWidth": req_cfg.get("frameWidth"),
                            "frameHeight": req_cfg.get("frameHeight"),
                            "cpu_percent": snap.get("cpu_percent"),
                            "ram_percent": snap.get("ram_percent"),
                            "gpu_percent": snap.get("gpu_percent"),
                            "gpu_temp_c": snap.get("gpu_temp_c"),
                            "gpu_power_w": snap.get("gpu_power_w"),
                            "pkg_power_w": snap.get("pkg_power_w"),
                            "sse_metrics_json": json.dumps(event.get("sse_metrics", {}), ensure_ascii=False),
                        }
                        rows.append(row)

                    append_rows(csv_path, fieldnames, rows)
                    total_captions += len(rows)

                    run_settings_payload = {
                        "timestamp_utc": utc_now_iso(),
                        "video_name": video_name,
                        "rtsp_url": rtsp_url,
                        "rest_request_key": req_key,
                        "model_name": model_name,
                        "run_id": run_info.run_id,
                        "pipeline_id": run_info.pipeline_id,
                        "service": service,
                        "execution": execution,
                        "collection": collection,
                        "effective_request_payload": payload,
                    }
                    settings_file = run_folder / settings_name
                    save_yaml(settings_file, run_settings_payload)

                    cooldown = int(execution.get("inter_run_cooldown_sec", 0))
                    if cooldown > 0:
                        time.sleep(cooldown)
    finally:
        metrics_collector.stop()

    print(
        f"[DONE] Completed runs: {total_runs}, captions captured: {total_captions}, skipped runs: {skipped_runs}"
    )
    if total_runs == 0 and skipped_runs > 0:
        raise SystemExit(
            "No runs started. The backend is reachable, but its pipeline server is unreachable. "
            "Start the full stack (without --no-stack) or ensure PIPELINE_SERVER_URL is resolvable from the backend."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark live video captioning runs")
    parser.add_argument(
        "--config",
        default="benchmarks/configs/fighting_config.yaml",
        help="Path to benchmark YAML config",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    if not config_path.exists():
        raise SystemExit(f"Config file not found: {config_path}")
    run_benchmark(config_path)


if __name__ == "__main__":
    main()
