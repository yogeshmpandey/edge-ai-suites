#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

VENV_DIR="${SCRIPT_DIR}/.venv-benchmark"
CONFIG_ARG="benchmarks/configs/fighting_config.yaml"
START_STACK="true"
COMPOSE_FILE="${PROJECT_ROOT}/compose.yaml"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -c, --config <path>       Benchmark config path (default: benchmarks/configs/fighting_config.yaml)
  -v, --venv <path>         Virtual env directory (default: benchmarks/.venv-benchmark)
  --no-stack                Do not run 'docker compose up -d'
  --compose-file <path>     Compose file path (default: <project>/compose.yaml)
  -h, --help                Show this help

Examples:
  ./benchmarks/run_benchmark.sh
  ./benchmarks/run_benchmark.sh --config benchmarks/configs/fighting_config.yaml
  ./benchmarks/run_benchmark.sh --no-stack
EOF
}

log() {
  printf '[benchmark] %s\n' "$*"
}

resolve_path() {
  local p="$1"
  if [[ -f "$p" ]]; then
    echo "$(cd "$(dirname "$p")" && pwd)/$(basename "$p")"
    return 0
  fi
  if [[ -f "${PROJECT_ROOT}/$p" ]]; then
    echo "${PROJECT_ROOT}/$p"
    return 0
  fi
  if [[ -f "${SCRIPT_DIR}/$p" ]]; then
    echo "${SCRIPT_DIR}/$p"
    return 0
  fi
  return 1
}

wait_for_http() {
  local url="$1"
  local timeout_sec="$2"
  local elapsed=0

  echo "Waiting for ${url} ..."
  until curl -fsS "$url" >/dev/null 2>&1; do
    sleep 2
    elapsed=$((elapsed + 2))
    if [[ "$elapsed" -ge "$timeout_sec" ]]; then
      echo "Timeout waiting for ${url}" >&2
      return 1
    fi
  done
  echo "Ready: ${url}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--config)
      CONFIG_ARG="$2"
      shift 2
      ;;
    -v|--venv)
      VENV_DIR="$2"
      shift 2
      ;;
    --no-stack)
      START_STACK="false"
      shift 1
      ;;
    --compose-file)
      COMPOSE_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

CONFIG_PATH="$(resolve_path "$CONFIG_ARG" || true)"
if [[ -z "${CONFIG_PATH}" ]]; then
  echo "Config file not found: ${CONFIG_ARG}" >&2
  exit 1
fi

echo "Project root: ${PROJECT_ROOT}"
echo "Config file: ${CONFIG_PATH}"
echo "Venv dir: ${VENV_DIR}"

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Creating benchmark venv at ${VENV_DIR}"
  python3 -m venv "${VENV_DIR}"
fi

source "${VENV_DIR}/bin/activate"

echo "Installing benchmark dependencies"
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r "${SCRIPT_DIR}/requirements-benchmark.txt"

if [[ "${START_STACK}" == "true" ]]; then
  if [[ ! -f "${COMPOSE_FILE}" ]]; then
    echo "Compose file not found: ${COMPOSE_FILE}" >&2
    exit 1
  fi
  echo "Starting benchmark stack with docker compose"
  (cd "${PROJECT_ROOT}" && docker compose -f "${COMPOSE_FILE}" up -d)

  wait_for_http "http://localhost:4173/api/health" 180
  wait_for_http "http://localhost:9090/health" 180
fi

echo "Running benchmark preflight checks"
python - "$CONFIG_PATH" <<'PY'
import json
import sys
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

import yaml

cfg_path = sys.argv[1]
with open(cfg_path, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

service = cfg.get("service", {})
api_base = str(service.get("api_base_url", "")).rstrip("/")
runs_endpoint = service.get("runs_endpoint", "/api/runs")
rest_keys = sorted([k for k in cfg.keys() if str(k).startswith("rest_request_")])
first_rest = cfg.get(rest_keys[0], {}) if rest_keys else {}

if not api_base:
    print("[ERROR] Missing service.api_base_url in config")
    sys.exit(1)

health_url = f"{api_base}/api/health"
try:
    with urlopen(health_url, timeout=10) as r:
        _ = r.read()
    print(f"[preflight] Backend reachable: {health_url}")
except Exception as exc:
    print(f"[ERROR] Backend not reachable at {health_url}: {exc}")
    sys.exit(1)

dummy_payload = {
    "rtspUrl": "rtsp://127.0.0.1:8554/preflight",
    "prompt": first_rest.get("prompt", "preflight"),
    "modelName": first_rest.get("modelName", "OpenGVLab/InternVL2-2B") or "OpenGVLab/InternVL2-2B",
    "maxNewTokens": first_rest.get("maxNewTokens", 1),
}
if first_rest.get("pipelineName"):
    dummy_payload["pipelineName"] = first_rest["pipelineName"]
req = Request(
    f"{api_base}{runs_endpoint}",
    method="POST",
    data=json.dumps(dummy_payload).encode("utf-8"),
    headers={"Content-Type": "application/json", "Accept": "application/json"},
)

try:
    started_run = None
    with urlopen(req, timeout=20) as r:
        body = r.read().decode("utf-8", errors="replace")
        try:
            started_run = json.loads(body)
        except Exception:
            started_run = None

    if isinstance(started_run, dict) and started_run.get("runId"):
        run_id = str(started_run["runId"])
        stop_req = Request(
            f"{api_base}{runs_endpoint}/{run_id}",
            method="DELETE",
            headers={"Accept": "application/json"},
        )
        try:
            with urlopen(stop_req, timeout=20) as _:
                pass
            print(f"[preflight] Stopped temporary preflight run: {run_id}")
        except Exception as stop_exc:
            print(f"[WARN] Could not stop temporary preflight run {run_id}: {stop_exc}")

        print("[preflight] Run-start endpoint accepted request")
    print("[preflight] Connectivity check passed")
except HTTPError as exc:
    detail = exc.read().decode("utf-8", errors="replace")
    if exc.code == 502 and "Pipeline not found" in detail:
        pipeline_name = dummy_payload.get("pipelineName", "<not-set>")
        print(f"[ERROR] Config pipelineName not found: {pipeline_name}")
        print("[ERROR] Use a valid name from GET /api/pipelines.")
        sys.exit(2)
    if exc.code == 502 and "Pipeline server unreachable" in detail:
        print("[ERROR] Backend reachable but pipeline server is unreachable from backend.")
        print("[ERROR] Start the full compose stack or fix PIPELINE_SERVER_URL DNS/network.")
        sys.exit(2)
    if exc.code in (400, 422):
        print("[preflight] Run-start endpoint reachable (request rejected as expected for dummy payload).")
        sys.exit(0)
    print(f"[ERROR] Unexpected HTTP {exc.code} from start-run preflight: {detail}")
    sys.exit(1)
except URLError as exc:
    print(f"[ERROR] Network error during start-run preflight: {exc}")
    sys.exit(1)
PY

export PYTHONUNBUFFERED=1

echo "Running benchmark"
cd "${PROJECT_ROOT}"
python "${SCRIPT_DIR}/run_caption_benchmark.py" --config "${CONFIG_PATH}"
