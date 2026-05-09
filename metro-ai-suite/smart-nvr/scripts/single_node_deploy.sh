#!/usr/bin/env bash
set -euo pipefail

RED=$'\033[0;31m'
GREEN=$'\033[0;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[0;34m'
NC=$'\033[0m'

log() {
  local level="$1"
  local color="$2"
  shift 2
  printf '%s %b[%s]%b %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$color" "$level" "$NC" "$*"
}

info() { log INFO "$BLUE" "$@"; }
warn() { log WARN "$YELLOW" "$@"; }
error() { log ERROR "$RED" "$@"; }
skip() { log SKIP "$GREEN" "$@"; }
die() {
  error "$@"
  exit 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMART_NVR_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
EDGE_AI_SUITES_ROOT="$(cd "${SMART_NVR_ROOT}/../.." && pwd)"
METRO_RECIPE_DIR="${METRO_RECIPE_DIR:-${SMART_NVR_ROOT}/../metro-vision-ai-app-recipe}"
METRO_RECIPE_DIR="$(cd "${METRO_RECIPE_DIR}" 2>/dev/null && pwd || true)"
SMART_INTERSECTION_DIR="${SMART_INTERSECTION_DIR:-${METRO_RECIPE_DIR}/smart-intersection}"
SOURCE="${SOURCE:-${SMART_INTERSECTION_DIR}/src}"

DEPLOY_STATE_DIR="${DEPLOY_STATE_DIR:-${SMART_NVR_ROOT}/.deploy-state/single-node}"
LOG_DIR="${DEPLOY_STATE_DIR}/logs"
BACKUP_DIR="${DEPLOY_STATE_DIR}/backups"
EDGE_AI_LIBRARIES_DIR="${EDGE_AI_LIBRARIES_DIR:-${DEPLOY_STATE_DIR}/edge-ai-libraries}"
VSS_APP_DIR="${EDGE_AI_LIBRARIES_DIR}/sample-applications/video-search-and-summarization"

EDGE_AI_LIBRARIES_URL="${EDGE_AI_LIBRARIES_URL:-https://github.com/open-edge-platform/edge-ai-libraries.git}"
EDGE_AI_LIBRARIES_COMMIT="${EDGE_AI_LIBRARIES_COMMIT:-7a27eab2ba3fe99baf59e45ff4d193f60011362a}"
VIDEO_BRANCH="${VIDEO_BRANCH:-main}"
VIDEO_URL="${VIDEO_URL:-https://github.com/open-edge-platform/edge-ai-resources/raw/refs/heads/${VIDEO_BRANCH}/videos}"

FRIGATE_HTTP_PORT="${FRIGATE_HTTP_PORT:-5000}"
FRIGATE_RTSP_HOST="${FRIGATE_RTSP_HOST:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
FRIGATE_RTSP_HOST="${FRIGATE_RTSP_HOST:-localhost}"
FRIGATE_RTSP_PORT="${FRIGATE_RTSP_PORT:-8554}"
SMARTNVR_API_PORT="${SMARTNVR_API_PORT:-8000}"
SMARTNVR_UI_PORT="${SMARTNVR_UI_PORT:-7860}"
SCENESCAPE_MQTT_PORT="${SCENESCAPE_MQTT_PORT:-1883}"
SMARTNVR_MQTT_PORT="${SMARTNVR_MQTT_PORT:-1884}"
VSS_SEARCH_PORT="${VSS_SEARCH_PORT:-12345}"

CAMERAS=(camera1 camera2 camera3 camera4)
REQUESTED_VIDEOS=(1122east_h264.ts 1122west_h264.ts 1122north_h264.ts 1122south_h264.ts)

CLEANUP=false
FORCE=false
SEED_DEMO_RULES=true

usage() {
  cat <<USAGE
Usage: bash scripts/single_node_deploy.sh [--cleanup] [--force] [--no-seed-rules] [--help]

Deploys a single-node SmartNVR + VSS search + Frigate + SceneScape demo.

Options:
  --cleanup        Stop demo stacks and restore generated SmartNVR config.
  --force          Re-render generated configs and restart services even if healthy.
  --no-seed-rules  Do not seed demo SceneScape rules.
  --help           Show this help text.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cleanup) CLEANUP=true ;;
    --force) FORCE=true ;;
    --no-seed-rules) SEED_DEMO_RULES=false ;;
    --help|-h) usage; exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
  shift
done

mkdir -p "$DEPLOY_STATE_DIR" "$LOG_DIR" "$BACKUP_DIR"

hash_file() {
  sha256sum "$1" | awk '{print $1}'
}

state_file() {
  printf '%s/%s.sha256' "$DEPLOY_STATE_DIR" "$1"
}

write_state_hash() {
  local name="$1"
  local file="$2"
  hash_file "$file" > "$(state_file "$name")"
}

state_hash_matches() {
  local name="$1"
  local file="$2"
  [[ -f "$(state_file "$name")" ]] && [[ "$(cat "$(state_file "$name")")" == "$(hash_file "$file")" ]]
}

backup_once() {
  local file="$1"
  local name="$2"
  if [[ -f "$file" && ! -f "${BACKUP_DIR}/${name}" ]]; then
    cp "$file" "${BACKUP_DIR}/${name}"
  fi
}

container_running() {
  local name="$1"
  [[ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" == "true" ]]
}

container_healthy_or_running() {
  local name="$1"
  local state
  state="$(docker inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}{{.State.Status}}{{end}}' "$name" 2>/dev/null || true)"
  [[ "$state" == "healthy" || "$state" == "running" ]]
}

scenescape_services_running() {
  [[ -f "${METRO_RECIPE_DIR}/docker-compose.yml" ]] || return 1
  local running
  running="$(cd "$METRO_RECIPE_DIR" && docker compose ps --services --filter status=running 2>/dev/null || true)"
  grep -qx 'broker' <<<"$running" &&
    grep -qx 'dlstreamer-pipeline-server' <<<"$running" &&
    grep -qx 'web' <<<"$running" &&
    grep -qx 'scene' <<<"$running"
}

compose_down_if_exists() {
  local compose_file="$1"
  local workdir="$2"
  if [[ -f "$compose_file" ]]; then
    (cd "$workdir" && docker compose -f "$compose_file" down) || warn "Failed to stop compose stack: $compose_file"
  fi
}

validate_docker() {
  info "Validating Docker prerequisites"
  command -v docker >/dev/null 2>&1 || die "Docker is not installed or not on PATH."
  docker compose version >/dev/null 2>&1 || die "Docker Compose v2 is not installed or not available as 'docker compose'."

  local log_file="${LOG_DIR}/docker-hello-world.log"
  if docker pull hello-world >"$log_file" 2>&1 && docker run --rm hello-world >>"$log_file" 2>&1; then
    info "Docker daemon and permissions verified"
    return
  fi

  if grep -qiE 'permission denied|Got permission denied|access denied' "$log_file"; then
    die "Docker permission denied. Add the current user to the docker group or run with appropriate privileges. See ${log_file}"
  fi
  if grep -qiE 'Cannot connect to the Docker daemon|Is the docker daemon running' "$log_file"; then
    die "Docker daemon is not running or not reachable. Start Docker and retry. See ${log_file}"
  fi
  if grep -qiE 'network|timeout|TLS handshake|proxy|no such host|connection refused|toomanyrequests' "$log_file"; then
    die "Docker works locally but pulling hello-world failed, likely due to network/registry/proxy settings. See ${log_file}"
  fi
  die "Docker validation failed. See ${log_file}"
}

ensure_python3() {
  command -v python3 >/dev/null 2>&1 || die "python3 is required for rendering JSON config and reading SceneScape credentials."
}

ensure_disk_space() {
  local available_kb
  available_kb="$(df -Pk "$SMART_NVR_ROOT" | awk 'NR==2 {print $4}')"
  if (( available_kb < 20971520 )); then
    warn "Less than 20 GiB free under ${SMART_NVR_ROOT}; VSS images/models may require more disk space."
  fi
}

export_vss_env() {
  export REGISTRY_URL="${REGISTRY_URL:-intel}"
  export TAG="${TAG:-latest}"
  export MINIO_ROOT_USER="${MINIO_ROOT_USER:-minio}"
  export MINIO_ROOT_PASSWORD="${MINIO_ROOT_PASSWORD:-minio_minio}"
  export POSTGRES_USER="${POSTGRES_USER:-postgres}"
  export POSTGRES_PASSWORD="${POSTGRES_PASSWORD:-postures}"
  export RABBITMQ_USER="${RABBITMQ_USER:-RabbitMQ}"
  export RABBITMQ_PASSWORD="${RABBITMQ_PASSWORD:-Rabbitpasse}"
  export VLM_COMPRESSION_WEIGHT_FORMAT="${VLM_COMPRESSION_WEIGHT_FORMAT:-int8}"
  export VLM_MODEL_NAME="${VLM_MODEL_NAME:-Qwen/Qwen2.5-VL-3B-Instruct}"
  export ENABLED_WHISPER_MODELS="${ENABLED_WHISPER_MODELS:-tiny.en,small.en,medium.en}"
  export OD_MODEL_NAME="${OD_MODEL_NAME:-yolov8l-worldv2}"
  export VCLIP_MODEL="${VCLIP_MODEL:-openai/clip-vit-base-patch32}"
  export QWEN_MODEL="${QWEN_MODEL:-Qwen/Qwen3-Embedding-0.6B}"
  export EMBEDDING_PROCESSING_MODE="${EMBEDDING_PROCESSING_MODE:-sdk}"
  export EMBEDDING_MODEL_NAME="${EMBEDDING_MODEL_NAME:-CLIP/clip-vit-b-32}"
  export SDK_MODEL_ID="${SDK_MODEL_ID:-CLIP/clip-vit-b-32}"
  export SDK_USE_OPENVINO="${SDK_USE_OPENVINO:-true}"
  export TEXT_EMBEDDING_MODEL_NAME="${TEXT_EMBEDDING_MODEL_NAME:-QwenText/qwen3-embedding-0.6b}"
  export ROI_CONSOLIDATION_ENABLED="${ROI_CONSOLIDATION_ENABLED:-true}"
  export ROI_CONSOLIDATION_IOU_THRESHOLD="${ROI_CONSOLIDATION_IOU_THRESHOLD:-0.3}"
  export ROI_CONSOLIDATION_CONTEXT_SCALE="${ROI_CONSOLIDATION_CONTEXT_SCALE:-0.35}"
  export ENABLE_VSS_COLLECTOR="${ENABLE_VSS_COLLECTOR:-true}"
  export FRAME_INTERVAL="${FRAME_INTERVAL:-15}"
  if [[ -n "${HUGGINGFACE_TOKEN:-}" ]]; then
    export HUGGINGFACE_TOKEN
  fi
}

vss_healthy() {
  curl -fsS "http://localhost:${VSS_SEARCH_PORT}/" >/dev/null 2>&1 ||
    curl -fsS "http://localhost:${VSS_SEARCH_PORT}/manager/health" >/dev/null 2>&1
}

deploy_vss() {
  info "Preparing VSS search mode at pinned commit ${EDGE_AI_LIBRARIES_COMMIT}"
  if [[ -d "${EDGE_AI_LIBRARIES_DIR}/.git" ]]; then
    (cd "$EDGE_AI_LIBRARIES_DIR" && git fetch --quiet origin "$EDGE_AI_LIBRARIES_COMMIT")
  else
    git clone "$EDGE_AI_LIBRARIES_URL" "$EDGE_AI_LIBRARIES_DIR"
  fi

  local current_commit dirty
  current_commit="$(cd "$EDGE_AI_LIBRARIES_DIR" && git rev-parse HEAD 2>/dev/null || true)"
  dirty="$(cd "$EDGE_AI_LIBRARIES_DIR" && git status --porcelain)"
  if [[ "$current_commit" != "$EDGE_AI_LIBRARIES_COMMIT" ]]; then
    [[ -z "$dirty" ]] || die "edge-ai-libraries has local changes at ${EDGE_AI_LIBRARIES_DIR}; commit/stash them or rerun with a clean clone."
    (cd "$EDGE_AI_LIBRARIES_DIR" && git checkout --quiet "$EDGE_AI_LIBRARIES_COMMIT")
  fi

  [[ -d "$VSS_APP_DIR" ]] || die "VSS app directory not found: ${VSS_APP_DIR}"

  if [[ "$FORCE" == false ]] && vss_healthy; then
    skip "VSS appears healthy on port ${VSS_SEARCH_PORT}"
    return
  fi

  export_vss_env
  local log_file="${LOG_DIR}/vss-setup-search.log"
  info "Starting VSS in search mode; logs: ${log_file}"
  if ! (
    set +eu
    cd "$VSS_APP_DIR"
    source ./setup.sh --search
  ) >"$log_file" 2>&1; then
    die "VSS setup failed. See ${log_file}"
  fi
  sleep 5
  if ! vss_healthy; then
    die "VSS setup completed but search endpoint on port ${VSS_SEARCH_PORT} is not reachable. See ${log_file}"
  fi
}

download_videos() {
  local dlstreamer_video_dir="${SOURCE}/dlstreamer-pipeline-server/videos"
  local nvr_video_dir="${SMART_NVR_ROOT}/resources/videos"
  mkdir -p "$dlstreamer_video_dir" "$nvr_video_dir"

  if [[ ! -d "$dlstreamer_video_dir" ]] || [[ -z "$(find "$dlstreamer_video_dir" -type f -name "*.ts" 2>/dev/null)" ]]; then
    info "Downloading demo videos"
    for video in "${REQUESTED_VIDEOS[@]}"; do
      if [[ ! -f "${nvr_video_dir}/${video}" ]]; then
        curl -fL "${VIDEO_URL}/${video}" -o "${nvr_video_dir}/${video}"
      fi
      cp -f "${nvr_video_dir}/${video}" "${dlstreamer_video_dir}/${video}"
    done
  else
    skip "DL Streamer video directory already contains .ts files"
    for video in "${REQUESTED_VIDEOS[@]}"; do
      if [[ ! -f "${nvr_video_dir}/${video}" && -f "${dlstreamer_video_dir}/${video}" ]]; then
        cp -f "${dlstreamer_video_dir}/${video}" "${nvr_video_dir}/${video}"
      fi
    done
  fi

  for video in "${REQUESTED_VIDEOS[@]}"; do
    [[ -f "${nvr_video_dir}/${video}" ]] || die "Missing expected demo video: ${nvr_video_dir}/${video}"
  done
  info "All demo videos verified in resources/videos"
}

render_frigate_config() {
  local target="${SMART_NVR_ROOT}/resources/frigate-config/config.yml"
  local source="${SMART_NVR_ROOT}/resources/frigate-config/config-scenescape.yml"
  backup_once "$target" "frigate-config.yml"

  cp "$source" "$target"
  write_state_hash "frigate-config" "$target"
}

render_scenescape_config() {
  local target="${SOURCE}/dlstreamer-pipeline-server/config.json"
  local template="${SMART_NVR_ROOT}/resources/si-rtsp-config.json"
  [[ -f "$template" ]] || die "SceneScape RTSP config template not found: ${template}"
  mkdir -p "$(dirname "$target")"
  backup_once "$target" "scenescape-dlstreamer-config.json"

  CONFIG_TARGET="$target" \
  CONFIG_TEMPLATE="$template" \
  RTSP_HOST="$FRIGATE_RTSP_HOST" \
  RTSP_PORT="$FRIGATE_RTSP_PORT" \
  python3 - <<'PY'
import json
import os

target = os.environ["CONFIG_TARGET"]
template = os.environ["CONFIG_TEMPLATE"]
rtsp_host = os.environ["RTSP_HOST"]
rtsp_port = os.environ["RTSP_PORT"]

def replace_placeholders(value):
    if isinstance(value, str):
        return (
            value
            .replace("{RTSP_STREAM_IP}", rtsp_host)
            .replace("{RTSP_STREAM_PORT}", rtsp_port)
        )
    if isinstance(value, list):
        return [replace_placeholders(item) for item in value]
    if isinstance(value, dict):
        return {key: replace_placeholders(item) for key, item in value.items()}
    return value

with open(template, encoding="utf-8") as fh:
    rendered = replace_placeholders(json.load(fh))

camera_ids = {
    pipeline["payload"]["parameters"]["camera_config"]["cameraid"]
    for pipeline in rendered["config"]["pipelines"]
}
expected = {"camera1", "camera2", "camera3", "camera4"}
if camera_ids != expected:
    raise SystemExit(f"Unexpected SceneScape camera IDs: {sorted(camera_ids)}")

with open(target, "w", encoding="utf-8") as fh:
    json.dump(rendered, fh, indent=4)
    fh.write("\n")
PY
  write_state_hash "scenescape-config" "$target"
}

prepare_scenescape_compose() {
  local compose_file="${METRO_RECIPE_DIR}/docker-compose.yml"
  [[ -f "$compose_file" ]] || die "docker-compose.yml not found at ${compose_file}. install.sh should have created it."
  backup_once "$compose_file" "docker-compose.yml"

  MQTT_HOST_PORT="$SCENESCAPE_MQTT_PORT" \
  python3 - "$compose_file" <<'PY'
import os
import sys
from pathlib import Path

mqtt_host_port = os.environ["MQTT_HOST_PORT"]
path = Path(sys.argv[1])
out = []
in_broker = False
broker_indent = ""
added_broker_port = False

for line in path.read_text(encoding="utf-8").splitlines():
    stripped = line.strip()

    # Track when we're inside the broker service block
    if stripped == "broker:":
        in_broker = True
        broker_indent = line[:len(line) - len(line.lstrip())]
    elif in_broker and stripped and not stripped.startswith("#"):
        # If current line is at the same indent level as "broker:", we've left the broker block
        current_indent = line[:len(line) - len(line.lstrip())]
        if len(current_indent) <= len(broker_indent) and stripped.endswith(":"):
            in_broker = False

    # Fix broker user: run as root to avoid /mosquitto/secrets permission error
    if in_broker and stripped == 'user: "${UID}:${GID}"':
        out.append('    user: "0:0"')
        continue

    # Add MQTT port to broker only if not already present
    if in_broker and not added_broker_port:
        if stripped.startswith('- "') and stripped.endswith(':1883"'):
            added_broker_port = True
    if in_broker and not added_broker_port and stripped.startswith("volumes:"):
        out.append('    ports:')
        out.append(f'      - "{mqtt_host_port}:1883"')
        added_broker_port = True

    # Remap pgserver host port to avoid conflict with local PostgreSQL on 5432
    if stripped == '- "5432:5432"':
        out.append(line.replace('"5432:5432"', '"5433:5432"'))
        continue

    out.append(line)

path.write_text("\n".join(out) + "\n", encoding="utf-8")
PY
  write_state_hash "scenescape-compose" "$compose_file"
}

prepare_scenescape() {
  info "Running install.sh smart-intersection to set up SceneScape assets"
  (
    cd "$METRO_RECIPE_DIR"
    ./install.sh smart-intersection "${FRIGATE_RTSP_HOST}"
  )
  render_scenescape_config
  prepare_scenescape_compose
}

start_scenescape() {
  if [[ "$FORCE" == false ]] && scenescape_services_running && state_hash_matches "scenescape-compose" "${METRO_RECIPE_DIR}/docker-compose.yml"; then
    skip "SceneScape containers already running with generated compose"
    return
  fi

  info "Starting SceneScape stack"
  (
    cd "$METRO_RECIPE_DIR"
    export GID="$(id -g)"
    export SUPASS
    SUPASS="$(cat "${SMART_INTERSECTION_DIR}/src/secrets/supass")" || die "Could not read supass from ${SMART_INTERSECTION_DIR}/src/secrets/supass"
    docker compose up -d
  )
}

read_scenescape_credentials() {
  local auth_file="${SOURCE}/secrets/browser.auth"
  [[ -f "$auth_file" ]] || die "SceneScape browser.auth not found at ${auth_file}"
  local creds
  creds="$(python3 - "$auth_file" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as fh:
    data = json.load(fh)
print(data.get("user", ""))
print(data.get("password", ""))
PY
)"
  SCENESCAPE_MQTT_USER="$(printf '%s\n' "$creds" | sed -n '1p')"
  SCENESCAPE_MQTT_PASSWORD="$(printf '%s\n' "$creds" | sed -n '2p')"
  [[ -n "$SCENESCAPE_MQTT_USER" && -n "$SCENESCAPE_MQTT_PASSWORD" ]] || die "Could not parse SceneScape MQTT credentials from ${auth_file}"
  export SCENESCAPE_MQTT_USER SCENESCAPE_MQTT_PASSWORD
}

deploy_smartnvr() {
  render_frigate_config
  read_scenescape_credentials

  export HOST_IP="${FRIGATE_RTSP_HOST}"
  export NVR_SCENESCAPE=true
  export NVR_GENAI=false
  export MQTT_USER="${MQTT_USER:-mqtt}"
  export MQTT_PASSWORD="${MQTT_PASSWORD:-mqtt}"
  export MQTT_PORT="${SMARTNVR_MQTT_PORT}"
  export VSS_SEARCH_IP="${VSS_SEARCH_IP:-${HOST_IP}}"
  export VSS_SEARCH_PORT="${VSS_SEARCH_PORT}"
  export VSS_SUMMARY_IP="${VSS_SUMMARY_IP:-${VSS_SEARCH_IP}}"
  export VSS_SUMMARY_PORT="${VSS_SUMMARY_PORT:-${VSS_SEARCH_PORT}}"
  export SCENESCAPE_MQTT_BROKER="${SCENESCAPE_MQTT_BROKER:-${HOST_IP}}"
  export SCENESCAPE_MQTT_PORT="${SCENESCAPE_MQTT_PORT}"
  export SCENESCAPE_THROTTLE_INTERVAL="${SCENESCAPE_THROTTLE_INTERVAL:-2.0}"
  export SCENESCAPE_CERTS_DIR="${SOURCE}/secrets/certs"
  export REGISTRY="${REGISTRY:-}"
  export TAG="${TAG:-latest}"
  export VLM_SERVING_IP="${VLM_SERVING_IP:-${HOST_IP}}"
  export VLM_SERVING_PORT="${VLM_SERVING_PORT:-9766}"

  if [[ "$FORCE" == false ]] && container_healthy_or_running "nvr-event-router" && container_healthy_or_running "frigate-vms" && state_hash_matches "frigate-config" "${SMART_NVR_ROOT}/resources/frigate-config/config.yml"; then
    skip "SmartNVR/Frigate already running with generated Frigate config"
    return
  fi

  info "Starting SmartNVR stack"
  (cd "$SMART_NVR_ROOT" && docker compose -f docker/compose.yaml up -d --build)
}

seed_demo_rules() {
  [[ "$SEED_DEMO_RULES" == true ]] || return
  info "Seeding demo SceneScape rules"
  for camera in "${CAMERAS[@]}"; do
    local legacy_rule_id="single-node-${camera}-pedestrian-search"
    if curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/${legacy_rule_id}" >/dev/null 2>&1; then
      curl -fsS -X DELETE "http://localhost:${SMARTNVR_API_PORT}/rules/${legacy_rule_id}" >/dev/null
      info "Removed legacy demo rule ${legacy_rule_id}"
    fi

    local rule_id="single-node-${camera}-vehicle-search"
    if curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/${rule_id}" >/dev/null 2>&1; then
      skip "Rule already exists: ${rule_id}"
      continue
    fi
    curl -fsS -X POST "http://localhost:${SMARTNVR_API_PORT}/rules/" \
      -H 'Content-Type: application/json' \
      -d "{\"id\":\"${rule_id}\",\"label\":\"vehicle\",\"action\":\"add to search\",\"camera\":\"${camera}\",\"source\":\"scenescape\",\"count\":1}" >/dev/null
    info "Seeded rule ${rule_id}"
  done
}

verify_vss() {
  vss_healthy
}

verify_frigate() {
  curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/api/go2rtc/streams" | grep -q "${CAMERAS[0]}" &&
  curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/api/go2rtc/streams" | grep -q "${CAMERAS[1]}" &&
  curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/api/go2rtc/streams" | grep -q "${CAMERAS[2]}" &&
  curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/api/go2rtc/streams" | grep -q "${CAMERAS[3]}"
}

verify_scenescape() {
  sleep 5
  scenescape_services_running || return 1
  docker exec metro-vision-ai-app-recipe-dlstreamer-pipeline-server-1 getent hosts broker.scenescape.intel.com >/dev/null || return 1
  # Pipeline status is behind nginx reverse proxy on the release-2026.0.0 stack
  curl -kfsS "https://localhost/api/pipelines/status" | python3 -c '
import json
import sys

pipelines = json.load(sys.stdin)
if len(pipelines) != 4:
    raise SystemExit(1)
if any(pipeline.get("state") != "RUNNING" for pipeline in pipelines):
    raise SystemExit(1)
'
}

verify_smartnvr() {
  curl -fsS "http://localhost:${SMARTNVR_API_PORT}/health" >/dev/null
}

print_status() {
  local name="$1"
  local ok="$2"
  if [[ "$ok" == true ]]; then
    printf '| %-28s | Running ✅ |\n' "$name"
  else
    printf '| %-28s | Failed ❌  |\n' "$name"
  fi
}

verify_all() {
  info "Verifying end-to-end deployment"
  local vss=false frigate=false scenescape=false smartnvr=false rules=false

  verify_vss && vss=true || warn "VSS search endpoint is not reachable at http://localhost:${VSS_SEARCH_PORT}"
  verify_frigate && frigate=true || warn "Frigate go2rtc streams are not all visible at http://localhost:${FRIGATE_HTTP_PORT}/api/go2rtc/streams"
  verify_scenescape && scenescape=true || warn "One or more SceneScape containers are not running"
  verify_smartnvr && smartnvr=true || warn "SmartNVR API health check failed"
  if [[ "$SEED_DEMO_RULES" == true ]] && curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/" | grep -q 'single-node-camera1'; then
    rules=true
  elif [[ "$SEED_DEMO_RULES" == false ]]; then
    rules=true
  else
    warn "Demo rules were not found in SmartNVR"
  fi

  printf '\n| Component                    | Status     |\n'
  printf '|------------------------------|------------|\n'
  print_status "VSS search" "$vss"
  print_status "Frigate RTSP/recording" "$frigate"
  print_status "SceneScape" "$scenescape"
  print_status "SmartNVR API" "$smartnvr"
  print_status "Demo rules" "$rules"

  if [[ "$vss" == true && "$frigate" == true && "$scenescape" == true && "$smartnvr" == true && "$rules" == true ]]; then
    info "Single-node demo deployment verified"
    info "Frigate RTSP streams:"
    for camera in "${CAMERAS[@]}"; do
      info "  rtsp://${FRIGATE_RTSP_HOST}:${FRIGATE_RTSP_PORT}/${camera}"
    done
    info "SmartNVR UI: http://${FRIGATE_RTSP_HOST}:${SMARTNVR_UI_PORT}"
    info "SceneScape UI: https://${FRIGATE_RTSP_HOST}"
    return
  fi

  die "Deployment verification failed. Review warnings above and logs in ${LOG_DIR}."
}

cleanup() {
  info "Cleaning up single-node demo deployment"
  compose_down_if_exists "${SMART_NVR_ROOT}/docker/compose.yaml" "$SMART_NVR_ROOT"
  if [[ -f "${METRO_RECIPE_DIR}/docker-compose.yml" ]]; then
    (cd "$METRO_RECIPE_DIR" && docker compose down) || warn "Failed to stop SceneScape docker compose stack"
  else
    compose_down_if_exists "${METRO_RECIPE_DIR}/compose-scenescape.yml" "$METRO_RECIPE_DIR"
  fi
  if [[ -f "${VSS_APP_DIR}/setup.sh" ]]; then
    (set +eu; cd "$VSS_APP_DIR"; source ./setup.sh --clean-data) >"${LOG_DIR}/vss-down.log" 2>&1 || warn "VSS shutdown returned non-zero; see ${LOG_DIR}/vss-down.log"
  fi
  if [[ -f "${BACKUP_DIR}/frigate-config.yml" ]]; then
    cp "${BACKUP_DIR}/frigate-config.yml" "${SMART_NVR_ROOT}/resources/frigate-config/config.yml"
  fi
  rm -rf "$EDGE_AI_LIBRARIES_DIR"
  rm -f "${DEPLOY_STATE_DIR}"/*.sha256
  info "Cleanup complete. Demo videos and SceneScape generated secrets were preserved."
}

main() {
  ensure_python3
  if [[ "$CLEANUP" == true ]]; then
    cleanup
    exit 0
  fi

  validate_docker
  ensure_disk_space
  deploy_vss
  download_videos
  prepare_scenescape
  deploy_smartnvr
  start_scenescape
  seed_demo_rules
  verify_all
}

main "$@"
