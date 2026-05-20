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
EDGE_AI_LIBRARIES_COMMIT="${EDGE_AI_LIBRARIES_COMMIT:-v2026.1.0-rc1}"
VIDEO_BRANCH="${VIDEO_BRANCH:-main}"
VIDEO_URL="${VIDEO_URL:-https://github.com/open-edge-platform/edge-ai-resources/raw/refs/heads/${VIDEO_BRANCH}/videos}"

FRIGATE_HTTP_PORT="${FRIGATE_HTTP_PORT:-5000}"
DETECTED_HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}' || true)"
LOCAL_HOST_IP="${DETECTED_HOST_IP}"
RTSP_STREAM_BIND_IP="${RTSP_STREAM_BIND_IP:-0.0.0.0}"
RTSP_STREAM_HOST="${RTSP_STREAM_HOST:-${FRIGATE_RTSP_HOST:-${DETECTED_HOST_IP}}}"
RTSP_STREAM_PORT="${RTSP_STREAM_PORT:-${FRIGATE_RTSP_PORT:-8554}}"
FRIGATE_RTSP_HOST="${FRIGATE_RTSP_HOST:-${RTSP_STREAM_HOST}}"
FRIGATE_RTSP_PORT="${FRIGATE_RTSP_PORT:-${RTSP_STREAM_PORT}}"
SMARTNVR_API_PORT="${SMARTNVR_API_PORT:-8000}"
SMARTNVR_UI_PORT="${SMARTNVR_UI_PORT:-7860}"
SCENESCAPE_MQTT_PORT="${SCENESCAPE_MQTT_PORT:-1883}"
SCENESCAPE_MQTT_TLS_HOSTNAME="${SCENESCAPE_MQTT_TLS_HOSTNAME:-broker.scenescape.intel.com}"
SMARTNVR_MQTT_PORT="${SMARTNVR_MQTT_PORT:-1884}"
VSS_SEARCH_PORT="${VSS_SEARCH_PORT:-12345}"
MEDIAMTX_PROJECT="${MEDIAMTX_PROJECT:-smartnvr-mediamtx}"
STREAMER_COMPOSE_FILE="${SMART_NVR_ROOT}/streamer/docker-compose.yml"
SMARTNVR_OVERRIDE_COMPOSE="${DEPLOY_STATE_DIR}/smartnvr-scenescape.override.yml"

CAMERAS=(camera1 camera2 camera3 camera4)
CAMERA_VIDEOS=(1122north_h264.ts 1122east_h264.ts 1122south_h264.ts 1122west_h264.ts)
REQUESTED_VIDEOS=(1122east_h264.ts 1122west_h264.ts 1122north_h264.ts 1122south_h264.ts)

ACTION=""
DEPLOY_MODE="${DEPLOY_MODE:-}"
DUAL_PHASE="${DUAL_PHASE:-${DUAL_SETUP_PHASE:-}}"
RI_MQTT_HOST="${RI_MQTT_HOST:-}"
RI_MQTT_PORT="${RI_MQTT_PORT:-}"

usage() {
  cat <<USAGE
Usage: ./single_node_deploy.sh [--setup | --run | --down | --cleanup | --help]

Deployment helper for SmartNVR + VSS Search + Metro AI (SceneScape) demo.
At startup, choose single-node or dual-node deployment. In dual mode, choose:
  setup1      System 1: Smart Intersection RI + MediaMTX RTSP streams
  setup2      System 2: VSS, SmartNVR, Frigate (consumes RTSP from System 1)

Commands (mutually exclusive):
  (default)    Run --setup followed by --run (full first-time deploy).
  --setup      First-time setup: validate prereqs, clone deps, download
               videos, render configs, and build Docker images.
               Does NOT start services.
  --run        Start all services (MediaMTX, VSS, Frigate, SmartNVR, SceneScape) and verify.
  --down       Gracefully stop all running services.
  --cleanup    Stop services, clean data, remove volumes & cloned repos.
  --help       Show this help message.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --setup|--run|--down|--cleanup)
      [[ -z "$ACTION" ]] || die "Flags --setup, --run, --down, and --cleanup are mutually exclusive."
      ACTION="$1"
      ;;
    --help|-h) usage; exit 0 ;;
    *) usage; die "Unknown argument: $1" ;;
  esac
  shift
done

mkdir -p "$DEPLOY_STATE_DIR" "$LOG_DIR" "$BACKUP_DIR"

prompt_choice() {
  local result_var="$1"
  local prompt="$2"
  local default="$3"
  local choices="$4"
  local input

  while true; do
    read -r -p "${prompt} (${choices}) [${default}]: " input || input="$default"
    input="${input:-$default}"
    input="${input,,}"
    if [[ " ${choices} " == *" ${input} "* ]]; then
      printf -v "$result_var" '%s' "$input"
      return
    fi
    warn "Invalid choice '${input}'. Expected one of: ${choices}"
  done
}

prompt_required() {
  local result_var="$1"
  local prompt="$2"
  local secret="${3:-false}"
  local input

  while true; do
    if [[ "$secret" == true ]]; then
      read -r -s -p "${prompt}: " input || input=""
      printf '\n'
    else
      read -r -p "${prompt}: " input || input=""
    fi
    if [[ -n "$input" ]]; then
      printf -v "$result_var" '%s' "$input"
      return
    fi
    warn "A value is required."
  done
}

select_deploy_target() {
  if [[ -z "$DEPLOY_MODE" ]]; then
    if [[ -t 0 ]]; then
      prompt_choice DEPLOY_MODE "Select deployment mode" "single" "single dual"
    else
      DEPLOY_MODE="single"
      info "No interactive terminal detected; defaulting deployment mode to single."
    fi
  fi
  DEPLOY_MODE="${DEPLOY_MODE,,}"

  case "$DEPLOY_MODE" in
    single) DUAL_PHASE="" ;;
    dual)
      if [[ -z "$DUAL_PHASE" ]]; then
        if [[ -t 0 ]]; then
          printf '\n%s\n' "  setup1 = Smart Intersection RI + MediaMTX RTSP streams (video source machine)"
          printf '%s\n\n' "  setup2 = VSS + Frigate + SmartNVR (analytics & recording machine)"
          prompt_choice DUAL_PHASE "Select dual-node phase for this machine" "setup1" "setup1 setup2"
        else
          die "DUAL_SETUP_PHASE must be set to setup1 or setup2 when DEPLOY_MODE=dual in non-interactive mode."
        fi
      fi
      DUAL_PHASE="${DUAL_PHASE,,}"
      [[ "$DUAL_PHASE" == "setup1" || "$DUAL_PHASE" == "setup2" ]] || die "Invalid dual-node phase '${DUAL_PHASE}'. Expected setup1 or setup2."
      ;;
    *) die "Invalid deployment mode '${DEPLOY_MODE}'. Expected single or dual." ;;
  esac

  export DEPLOY_MODE DUAL_PHASE
}

validate_tcp_port() {
  local name="$1"
  local value="$2"
  [[ "$value" =~ ^[0-9]+$ ]] || die "${name} must be numeric; got '${value}'."
  (( value > 0 && value <= 65535 )) || die "${name} must be between 1 and 65535; got '${value}'."
}

validate_host_value() {
  local name="$1"
  local value="$2"
  [[ -n "$value" ]] || die "${name} cannot be empty."
  [[ "$value" =~ ^[A-Za-z0-9_.-]+$ ]] || die "${name} contains unsupported characters: '${value}'."
  case "$value" in
    localhost|127.*|0.0.0.0)
      die "${name} must be a LAN-reachable IP or DNS name, not '${value}'."
      ;;
  esac
}

resolve_host_ip() {
  local host="$1"
  if [[ "$host" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    printf '%s\n' "$host"
    return
  fi

  local resolved
  resolved="$(getent hosts "$host" 2>/dev/null | awk 'NR == 1 {print $1}')"
  [[ -n "$resolved" ]] || die "Could not resolve ${host}; use an IP address for RI_MQTT_HOST or add DNS/hosts resolution."
  printf '%s\n' "$resolved"
}

resolve_ri_mqtt_endpoint() {
  RI_MQTT_HOST="${RI_MQTT_HOST:-${DETECTED_HOST_IP:-${RTSP_STREAM_HOST}}}"
  RI_MQTT_PORT="${RI_MQTT_PORT:-${SCENESCAPE_MQTT_PORT}}"
  validate_host_value "RI_MQTT_HOST" "$RI_MQTT_HOST"
  validate_tcp_port "RI_MQTT_PORT" "$RI_MQTT_PORT"
  export RI_MQTT_HOST RI_MQTT_PORT
}

require_ri_mqtt_settings() {
  info "RI_MQTT_HOST and RI_MQTT_PORT come from System 1 and are required so System 2 connects to the same SceneScape MQTT broker."
  if [[ -z "$RI_MQTT_HOST" ]]; then
    [[ -t 0 ]] || die "RI_MQTT_HOST is required for dual/setup2. Copy it from the System 1 handoff block."
    prompt_required RI_MQTT_HOST "Enter RI_MQTT_HOST from System 1"
  fi
  if [[ -z "$RI_MQTT_PORT" ]]; then
    [[ -t 0 ]] || die "RI_MQTT_PORT is required for dual/setup2. Copy it from the System 1 handoff block."
    prompt_required RI_MQTT_PORT "Enter RI_MQTT_PORT from System 1"
  fi
  validate_host_value "RI_MQTT_HOST" "$RI_MQTT_HOST"
  validate_tcp_port "RI_MQTT_PORT" "$RI_MQTT_PORT"
  export RI_MQTT_HOST RI_MQTT_PORT
  export SCENESCAPE_MQTT_BROKER="$RI_MQTT_HOST"
  export SCENESCAPE_MQTT_PORT="$RI_MQTT_PORT"
}

require_ri_rtsp_settings() {
  info "RTSP_STREAM_HOST and RTSP_STREAM_PORT come from System 1 (MediaMTX) and are required so System 2 Frigate can consume the camera streams."
  if [[ -z "${RTSP_STREAM_HOST:-}" || "$RTSP_STREAM_HOST" == "$DETECTED_HOST_IP" ]]; then
    if [[ -t 0 ]]; then
      prompt_required RTSP_STREAM_HOST "Enter RTSP_STREAM_HOST from System 1"
    else
      die "RTSP_STREAM_HOST is required for dual/setup2. Copy it from the System 1 handoff block."
    fi
  fi
  if [[ -z "${RTSP_STREAM_PORT:-}" ]]; then
    RTSP_STREAM_PORT="8554"
  fi
  validate_host_value "RTSP_STREAM_HOST" "$RTSP_STREAM_HOST"
  validate_tcp_port "RTSP_STREAM_PORT" "$RTSP_STREAM_PORT"
  FRIGATE_RTSP_HOST="$RTSP_STREAM_HOST"
  FRIGATE_RTSP_PORT="$RTSP_STREAM_PORT"
  export RTSP_STREAM_HOST RTSP_STREAM_PORT FRIGATE_RTSP_HOST FRIGATE_RTSP_PORT
}

ensure_dual_setup1_rtsp_local() {
  info "System 1 will run MediaMTX locally and provide RTSP streams to System 2."
  RTSP_STREAM_HOST="${RTSP_STREAM_HOST:-${DETECTED_HOST_IP}}"
  FRIGATE_RTSP_HOST="$RTSP_STREAM_HOST"
  FRIGATE_RTSP_PORT="$RTSP_STREAM_PORT"
  validate_rtsp_settings
  if [[ "$RTSP_STREAM_BIND_IP" == "127."* || "$RTSP_STREAM_BIND_IP" == "localhost" ]]; then
    die "RTSP_STREAM_BIND_IP='${RTSP_STREAM_BIND_IP}' is not externally reachable. Use 0.0.0.0 for dual-node deployment."
  fi
}

print_connection_handoff() {
  resolve_ri_mqtt_endpoint
  cat <<HANDOFF

==================== System 1 connection handoff ====================
Copy these values to System 2 before running dual/setup2:
RI_MQTT_HOST=${RI_MQTT_HOST}
RI_MQTT_PORT=${RI_MQTT_PORT}
RTSP_STREAM_HOST=${RTSP_STREAM_HOST}
RTSP_STREAM_PORT=${RTSP_STREAM_PORT}
=====================================================================

HANDOFF
}

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

validate_rtsp_settings() {
  case "$RTSP_STREAM_HOST" in
    ""|localhost|127.*|0.0.0.0)
      die "RTSP_STREAM_HOST must be this host's LAN IP or DNS name reachable from containers. Detected '${RTSP_STREAM_HOST:-empty}'."
      ;;
  esac
  [[ "$RTSP_STREAM_PORT" =~ ^[0-9]+$ ]] || die "RTSP_STREAM_PORT must be numeric; got '${RTSP_STREAM_PORT}'."
}

wait_for_rtsp_streams() {
  info "Waiting for MediaMTX RTSP streams"
  local retries=30
  while (( retries > 0 )); do
    local ready=true
    local camera
    for camera in "${CAMERAS[@]}"; do
      if ! docker exec rtsp-publisher sh -c "timeout 10 ffprobe -v error -rtsp_transport tcp -select_streams v:0 -show_entries stream=codec_type -of csv=p=0 'rtsp://mediamtx:8554/${camera}' | grep -q video" >/dev/null 2>&1; then
        ready=false
        break
      fi
    done
    if [[ "$ready" == true ]]; then
      info "MediaMTX is serving all RTSP streams"
      return
    fi
    sleep 3
    (( retries-- ))
  done
  die "MediaMTX did not start serving all camera streams. Check docker logs rtsp-publisher and mediamtx."
}

start_rtsp_streamer() {
  validate_rtsp_settings
  local video_dir="${SMART_NVR_ROOT}/resources/videos"
  local i
  [[ -f "$STREAMER_COMPOSE_FILE" ]] || die "MediaMTX streamer compose file not found: ${STREAMER_COMPOSE_FILE}"
  for i in "${!CAMERA_VIDEOS[@]}"; do
    [[ -f "${video_dir}/${CAMERA_VIDEOS[$i]}" ]] || die "Missing demo video for ${CAMERAS[$i]}: ${video_dir}/${CAMERA_VIDEOS[$i]}. Run --setup first."
  done

  if docker port frigate-vms 8554/tcp >/dev/null 2>&1; then
    info "Stopping existing Frigate RTSP port mapping before starting MediaMTX"
    (cd "$SMART_NVR_ROOT" && docker compose -f docker/compose.yaml stop frigate) || docker stop frigate-vms >/dev/null
  fi

  info "Starting MediaMTX RTSP server on ${RTSP_STREAM_BIND_IP}:${RTSP_STREAM_PORT}"
  (cd "$SMART_NVR_ROOT" && RTSP_STREAM_BIND_IP="$RTSP_STREAM_BIND_IP" RTSP_STREAM_PORT="$RTSP_STREAM_PORT" docker compose -p "$MEDIAMTX_PROJECT" -f "$STREAMER_COMPOSE_FILE" up -d)
  wait_for_rtsp_streams
}

stop_rtsp_streamer() {
  if [[ -f "$STREAMER_COMPOSE_FILE" ]]; then
    (cd "$SMART_NVR_ROOT" && docker compose -p "$MEDIAMTX_PROJECT" -f "$STREAMER_COMPOSE_FILE" down) || warn "Failed to stop MediaMTX RTSP stack"
    return
  fi
  docker rm -f rtsp-publisher mediamtx >/dev/null 2>&1 || true
}

export_vss_env() {
  export REGISTRY_URL="${REGISTRY_URL:-intel}"
  export TAG="${TAG:-2026.1.0-rc1}"
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
  export MULTIMODAL_EMBEDDING_MODEL="${EMBEDDING_MODEL_NAME:-CLIP/clip-vit-b-32}"
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

clone_vss_repo() {
  info "Cloning/updating VSS repo at pinned ref ${EDGE_AI_LIBRARIES_COMMIT}"
  if [[ -d "${EDGE_AI_LIBRARIES_DIR}/.git" ]]; then
    (cd "$EDGE_AI_LIBRARIES_DIR" && git fetch --quiet --tags origin)
  else
    git clone "$EDGE_AI_LIBRARIES_URL" "$EDGE_AI_LIBRARIES_DIR"
  fi

  local current_commit target_commit dirty
  current_commit="$(cd "$EDGE_AI_LIBRARIES_DIR" && git rev-parse HEAD 2>/dev/null || true)"
  target_commit="$(cd "$EDGE_AI_LIBRARIES_DIR" && git rev-parse "${EDGE_AI_LIBRARIES_COMMIT}^{commit}" 2>/dev/null || true)"
  dirty="$(cd "$EDGE_AI_LIBRARIES_DIR" && git status --porcelain)"
  if [[ "$current_commit" != "$target_commit" ]]; then
    [[ -z "$dirty" ]] || die "edge-ai-libraries has local changes at ${EDGE_AI_LIBRARIES_DIR}; commit/stash them or rerun with a clean clone."
    (cd "$EDGE_AI_LIBRARIES_DIR" && git checkout --quiet "$EDGE_AI_LIBRARIES_COMMIT")
  fi

  [[ -d "$VSS_APP_DIR" ]] || die "VSS app directory not found: ${VSS_APP_DIR}"
}

start_vss() {
  [[ -d "$VSS_APP_DIR" ]] || die "VSS app directory not found: ${VSS_APP_DIR}. Run --setup first."

  if vss_healthy; then
    skip "VSS appears healthy on port ${VSS_SEARCH_PORT}"
    return
  fi

  export_vss_env
  local log_file="${LOG_DIR}/vss-setup-search.log"
  info "Starting VSS in search mode; For First time setup this will take ~15 min"
  info "logs: ${log_file}"
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

  validate_rtsp_settings
  CONFIG_TARGET="$target" \
  CONFIG_TEMPLATE="$source" \
  RTSP_HOST="$RTSP_STREAM_HOST" \
  RTSP_PORT="$RTSP_STREAM_PORT" \
  python3 - <<'PY'
import os
from pathlib import Path

target = Path(os.environ["CONFIG_TARGET"])
template = Path(os.environ["CONFIG_TEMPLATE"])
content = template.read_text(encoding="utf-8")
content = (
    content
    .replace("{RTSP_STREAM_IP}", os.environ["RTSP_HOST"])
    .replace("{RTSP_STREAM_PORT}", os.environ["RTSP_PORT"])
)
target.write_text(content, encoding="utf-8")
PY
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
  RTSP_HOST="$RTSP_STREAM_HOST" \
  RTSP_PORT="$RTSP_STREAM_PORT" \
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

  MQTT_HOST_PORT="${RI_MQTT_PORT:-$SCENESCAPE_MQTT_PORT}" \
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

    # Add or update MQTT port on the broker.
    if in_broker and not added_broker_port:
        if stripped.startswith('- "') and stripped.endswith(':1883"'):
            indent = line[:len(line) - len(line.lstrip())]
            out.append(f'{indent}- "{mqtt_host_port}:1883"')
            added_broker_port = True
            continue
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
    ./install.sh smart-intersection "${RTSP_STREAM_HOST}"
  )
  render_scenescape_config
  prepare_scenescape_compose
}

start_scenescape() {
  info "Starting SceneScape stack"
  (
    cd "$METRO_RECIPE_DIR"
    export GID="$(id -g)"
    export SUPASS="${SUPASS}"
    docker compose up -d
  )
}

export_smartnvr_env() {
  export HOST_IP="${LOCAL_HOST_IP}"
  export NVR_SCENESCAPE=true
  export NVR_GENAI=false
  export MQTT_USER="${MQTT_USER:-mqtt}"
  export MQTT_PASSWORD="${MQTT_PASSWORD:-mqtt}"
  export MQTT_PORT="${SMARTNVR_MQTT_PORT}"
  export VSS_SEARCH_IP="${VSS_SEARCH_IP:-${HOST_IP}}"
  export VSS_SEARCH_PORT="${VSS_SEARCH_PORT}"
  export VSS_SUMMARY_IP="${VSS_SUMMARY_IP:-${VSS_SEARCH_IP}}"
  export VSS_SUMMARY_PORT="${VSS_SUMMARY_PORT:-${VSS_SEARCH_PORT}}"
  export SCENESCAPE_MQTT_BROKER="${SCENESCAPE_MQTT_BROKER:-${SCENESCAPE_MQTT_TLS_HOSTNAME}}"
  export SCENESCAPE_MQTT_PORT="${SCENESCAPE_MQTT_PORT}"
  export SCENESCAPE_THROTTLE_INTERVAL="${SCENESCAPE_THROTTLE_INTERVAL:-2.0}"
  export REGISTRY="${REGISTRY:-}"
  export TAG="${TAG:-2026.1.0-rc1}"
  export VLM_SERVING_IP="${VLM_SERVING_IP:-${HOST_IP}}"
  export VLM_SERVING_PORT="${VLM_SERVING_PORT:-9766}"
}

render_smartnvr_scenescape_override() {
  local broker_target_host="${RI_MQTT_HOST:-${HOST_IP}}"
  local broker_target_ip
  broker_target_ip="$(resolve_host_ip "$broker_target_host")"
  mkdir -p "$(dirname "$SMARTNVR_OVERRIDE_COMPOSE")"
  cat >"$SMARTNVR_OVERRIDE_COMPOSE" <<YAML
services:
  nvr-event-router:
    extra_hosts:
      - "${SCENESCAPE_MQTT_TLS_HOSTNAME}:${broker_target_ip}"
YAML
}

smartnvr_compose_up() {
  local compose_args=(-f docker/compose.yaml)
  render_smartnvr_scenescape_override
  compose_args+=(-f "$SMARTNVR_OVERRIDE_COMPOSE")
  (cd "$SMART_NVR_ROOT" && docker compose "${compose_args[@]}" up -d "$@")
}

deploy_frigate() {
  render_frigate_config
  export_smartnvr_env

  info "Starting Frigate, MQTT broker, and Redis"
  (cd "$SMART_NVR_ROOT" && docker compose -f docker/compose.yaml up -d frigate mqtt-broker redis)

  info "Waiting for Frigate HTTP API to be ready"
  local retries=30
  while (( retries > 0 )); do
    if curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/" >/dev/null 2>&1; then
      info "Frigate is healthy"
      break
    fi
    sleep 5
    (( retries-- ))
  done
  if (( retries == 0 )); then
    warn "Frigate health check timed out — continuing anyway"
    return
  fi

  info "Waiting for Frigate camera ingest"
  retries=30
  while (( retries > 0 )); do
    if verify_frigate; then
      info "Frigate is ingesting all camera streams"
      return
    fi
    sleep 5
    (( retries-- ))
  done
  warn "Frigate camera ingest check timed out — continuing anyway"
}

deploy_smartnvr() {
  export_smartnvr_env

  info "Starting SmartNVR stack"
  smartnvr_compose_up
}

seed_demo_rules() {
  sleep 5
  info "Seeding demo SceneScape rules"

  # Only seed when no rules exist at all
  local existing_rules
  existing_rules=$(curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/" 2>/dev/null || echo "[]")
  if echo "$existing_rules" | grep -q '"id"'; then
    skip "Rules already exist — skipping seed"
    return
  fi

  # Assign a count between 3 and 5 for each camera (cycling 3,4,5,3,…)
  local counts=(3 4 5 3)
  local i=0
  for camera in "${CAMERAS[@]}"; do
    local rule_id="add to search-${camera}-vehicle-search"
    local count="${counts[$i]}"
    curl -fsS -X POST "http://localhost:${SMARTNVR_API_PORT}/rules/" \
      -H 'Content-Type: application/json' \
      -d "{\"id\":\"${rule_id}\",\"label\":\"vehicle\",\"action\":\"add to search\",\"camera\":\"${camera}\",\"source\":\"scenescape\",\"count\":${count}}" >/dev/null
    info "Seeded rule ${rule_id} (count=${count})"
    (( i++ )) || true
  done
}

verify_vss() {
  vss_healthy
}

verify_frigate() {
  sleep 5
  local cameras_csv
  cameras_csv="$(IFS=,; echo "${CAMERAS[*]}")"
  curl -fsS "http://localhost:${FRIGATE_HTTP_PORT}/api/stats" | CAMERAS_CSV="$cameras_csv" python3 -c '
import json
import os
import sys

stats = json.load(sys.stdin)
cameras = stats.get("cameras", {})
missing = []
for camera in os.environ["CAMERAS_CSV"].split(","):
    camera_stats = cameras.get(camera, {})
    try:
        fps = float(camera_stats.get("camera_fps") or 0)
    except (TypeError, ValueError):
        fps = 0
    if fps <= 0:
        missing.append(camera)
if missing:
    raise SystemExit(f"Frigate camera_fps not ready for: {missing}")
'
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

verify_ri_mqtt_endpoint() {
  local host="$1"
  local port="$2"
  timeout 5 bash -c ':</dev/tcp/"$0"/"$1"' "$host" "$port" >/dev/null 2>&1
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
  verify_frigate && frigate=true || warn "Frigate is not ingesting all MediaMTX camera streams"
  verify_scenescape && scenescape=true || warn "One or more SceneScape containers are not running"
  verify_smartnvr && smartnvr=true || warn "SmartNVR API health check failed"
  if curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/" | grep -q 'add to search'; then
    rules=true
  else
    warn "Demo rules were not found in SmartNVR"
  fi

  printf '\n| Component                    | Status     |\n'
  printf '|------------------------------|------------|\n'
  print_status "VSS search" "$vss"
  print_status "Frigate recording" "$frigate"
  print_status "SceneScape" "$scenescape"
  print_status "SmartNVR API" "$smartnvr"
  print_status "Demo rules" "$rules"

  if [[ "$vss" == true && "$frigate" == true && "$scenescape" == true && "$smartnvr" == true && "$rules" == true ]]; then
    info "Single-node demo deployment verified"
    info "Live Camera streams:"
    for camera in "${CAMERAS[@]}"; do
      info "  rtsp://${RTSP_STREAM_HOST}:${RTSP_STREAM_PORT}/${camera}"
    done
    info "Important URLs: "
    info "  NVR UI: http://${RTSP_STREAM_HOST}:${FRIGATE_HTTP_PORT}"
    info "  SmartNVR UI: http://${RTSP_STREAM_HOST}:${SMARTNVR_UI_PORT}"
    info "  Intel VSS Search UI: http://${RTSP_STREAM_HOST}:${VSS_SEARCH_PORT}"
    info "  Scenescape UI: https://${RTSP_STREAM_HOST}"
    info "Scenescape login username: admin"
    info "Scenescape login password: ${SUPASS}"
    return
  fi

  die "Deployment verification failed. Review warnings above and logs in ${LOG_DIR}."
}

verify_dual_setup1() {
  info "Verifying Smart Intersection RI + MediaMTX deployment"
  local scenescape=false mqtt=false rtsp=false

  scenescape_services_running && scenescape=true || warn "One or more SceneScape containers are not running"
  resolve_ri_mqtt_endpoint
  verify_ri_mqtt_endpoint "127.0.0.1" "$RI_MQTT_PORT" && mqtt=true || warn "SceneScape MQTT broker is not reachable on localhost:${RI_MQTT_PORT}"
  if docker ps --format '{{.Names}}' | grep -q mediamtx; then
    rtsp=true
  else
    warn "MediaMTX container is not running"
  fi

  printf '\n| Component                    | Status     |\n'
  printf '|------------------------------|------------|\n'
  print_status "Smart Intersection RI" "$scenescape"
  print_status "RI MQTT broker" "$mqtt"
  print_status "MediaMTX RTSP streams" "$rtsp"

  [[ "$scenescape" == true && "$mqtt" == true && "$rtsp" == true ]] || die "System 1 deployment verification failed. Review warnings above and logs in ${LOG_DIR}."
  info "System 1 Smart Intersection RI + MediaMTX deployment verified"
  info "RTSP streams available at:"
  for camera in "${CAMERAS[@]}"; do
    info "  rtsp://${RTSP_STREAM_HOST}:${RTSP_STREAM_PORT}/${camera}"
  done
  info "Important URLs:"
  info "  Scenescape UI: https://${RTSP_STREAM_HOST}"
  info "Scenescape login username: admin"
  info "Scenescape login password: ${SUPASS}"
}

verify_dual_setup2() {
  info "Verifying System 2 deployment"
  local vss=false frigate=false smartnvr=false rules=false mqtt=false rtsp=false

  verify_vss && vss=true || warn "VSS search endpoint is not reachable at http://localhost:${VSS_SEARCH_PORT}"
  verify_frigate && frigate=true || warn "Frigate is not ingesting camera streams from System 1 MediaMTX"
  verify_smartnvr && smartnvr=true || warn "SmartNVR API health check failed"
  verify_ri_mqtt_endpoint "$RI_MQTT_HOST" "$RI_MQTT_PORT" && mqtt=true || warn "System 1 MQTT broker is not reachable at ${RI_MQTT_HOST}:${RI_MQTT_PORT}"
  timeout 5 bash -c ':</dev/tcp/"$0"/"$1"' "$RTSP_STREAM_HOST" "$RTSP_STREAM_PORT" >/dev/null 2>&1 && rtsp=true || warn "System 1 MediaMTX RTSP is not reachable at ${RTSP_STREAM_HOST}:${RTSP_STREAM_PORT}"
  if curl -fsS "http://localhost:${SMARTNVR_API_PORT}/rules/" | grep -q 'add to search'; then
    rules=true
  else
    warn "Demo rules were not found in SmartNVR"
  fi

  printf '\n| Component                    | Status     |\n'
  printf '|------------------------------|------------|\n'
  print_status "VSS search" "$vss"
  print_status "Frigate recording" "$frigate"
  print_status "System 1 MQTT" "$mqtt"
  print_status "System 1 RTSP (MediaMTX)" "$rtsp"
  print_status "SmartNVR API" "$smartnvr"
  print_status "Demo rules" "$rules"

  if [[ "$vss" == true && "$frigate" == true && "$mqtt" == true && "$rtsp" == true && "$smartnvr" == true && "$rules" == true ]]; then
    info "System 2 deployment verified"
    info "Camera streams (from System 1):"
    for camera in "${CAMERAS[@]}"; do
      info "  rtsp://${RTSP_STREAM_HOST}:${RTSP_STREAM_PORT}/${camera}"
    done
    info "Important URLs: "
    info "  NVR UI: http://${LOCAL_HOST_IP}:${FRIGATE_HTTP_PORT}"
    info "  SmartNVR UI: http://${LOCAL_HOST_IP}:${SMARTNVR_UI_PORT}"
    info "  Intel VSS Search UI: http://${LOCAL_HOST_IP}:${VSS_SEARCH_PORT}"
    return
  fi

  die "System 2 deployment verification failed. Review warnings above and logs in ${LOG_DIR}."
}

do_down() {
  info "Stopping all services"
  compose_down_if_exists "${SMART_NVR_ROOT}/docker/compose.yaml" "$SMART_NVR_ROOT"
  stop_rtsp_streamer
  if [[ -f "${METRO_RECIPE_DIR}/docker-compose.yml" ]]; then
    (cd "$METRO_RECIPE_DIR" && docker compose down) || warn "Failed to stop SceneScape docker compose stack"
  else
    compose_down_if_exists "${METRO_RECIPE_DIR}/compose-scenescape.yml" "$METRO_RECIPE_DIR"
  fi
  if [[ -f "${VSS_APP_DIR}/setup.sh" ]]; then
    (set +eu; cd "$VSS_APP_DIR"; source ./setup.sh --down) >"${LOG_DIR}/vss-down.log" 2>&1 || warn "VSS shutdown returned non-zero; see ${LOG_DIR}/vss-down.log"
  fi
  if [[ -f "${BACKUP_DIR}/frigate-config.yml" ]]; then
    cp "${BACKUP_DIR}/frigate-config.yml" "${SMART_NVR_ROOT}/resources/frigate-config/config.yml"
  fi
  info "All services stopped."
}

do_cleanup() {
  do_down
  info "Removing data, cloned repos, and deploy state"
  if [[ -f "${VSS_APP_DIR}/setup.sh" ]]; then
    (set +eu; cd "$VSS_APP_DIR"; source ./setup.sh --clean-data) >"${LOG_DIR}/vss-cleanup.log" 2>&1 || warn "VSS clean-data returned non-zero; see ${LOG_DIR}/vss-cleanup.log"
  fi
  # Clean SceneScape Docker volumes
  local ss_volumes
  ss_volumes=$(docker volume ls -q --filter 'name=metro-vision-ai-app-recipe*' 2>/dev/null || true)
  if [[ -n "$ss_volumes" ]]; then
    info "Removing metro-vision-ai-app-recipe Docker volumes"
    echo "$ss_volumes" | xargs docker volume rm 2>/dev/null || warn "Some SceneScape volumes could not be removed"
  fi
  rm -f "${DEPLOY_STATE_DIR}"/*.sha256
  info "Cleanup complete. Demo videos and SceneScape generated secrets were preserved."
}

run_setup() {
  info "=== First-time setup ==="
  validate_docker
  validate_rtsp_settings
  ensure_disk_space
  clone_vss_repo
  download_videos
  prepare_scenescape
  info "Building SmartNVR Docker image"
  (cd "$SMART_NVR_ROOT" && ./build.sh)
  info "=== Setup complete. Run with --run to start the stack. ==="
}

run_setup_dual_setup1() {
  info "=== Dual-node setup1: Smart Intersection RI + MediaMTX setup ==="
  resolve_ri_mqtt_endpoint
  validate_docker
  ensure_dual_setup1_rtsp_local
  ensure_disk_space
  download_videos
  export SCENESCAPE_MQTT_PORT="$RI_MQTT_PORT"
  prepare_scenescape
  info "=== System 1 setup complete. Run with --run to start MediaMTX + Smart Intersection RI. ==="
  print_connection_handoff
}

run_setup_dual_setup2() {
  info "=== Dual-node setup2: VSS, SmartNVR, Frigate setup ==="
  require_ri_mqtt_settings
  require_ri_rtsp_settings
  validate_docker
  ensure_disk_space
  clone_vss_repo
  render_frigate_config
  info "Building SmartNVR Docker image"
  (cd "$SMART_NVR_ROOT" && ./build.sh)
  info "=== System 2 setup complete. Run with --run to start VSS, Frigate, and SmartNVR. ==="
}

read_supass() {
  [[ -f "${SMART_INTERSECTION_DIR}/src/secrets/supass" ]] || die "SUPASS file not found at ${SMART_INTERSECTION_DIR}/src/secrets/supass"
  export SUPASS="$(cat "${SMART_INTERSECTION_DIR}/src/secrets/supass")"
}

run_run() {
  info "=== Starting full stack ==="
  read_supass
  start_rtsp_streamer
  start_vss
  deploy_frigate
  start_scenescape
  deploy_smartnvr
  seed_demo_rules
  verify_all
}

run_run_dual_setup1() {
  info "=== Starting dual-node setup1: MediaMTX + Smart Intersection RI ==="
  resolve_ri_mqtt_endpoint
  ensure_dual_setup1_rtsp_local
  export SCENESCAPE_MQTT_PORT="$RI_MQTT_PORT"
  start_rtsp_streamer
  read_supass
  start_scenescape
  verify_dual_setup1
  print_connection_handoff
}

run_run_dual_setup2() {
  info "=== Starting dual-node setup2: VSS, Frigate, SmartNVR ==="
  require_ri_mqtt_settings
  require_ri_rtsp_settings
  start_vss
  deploy_frigate
  deploy_smartnvr
  seed_demo_rules
  verify_dual_setup2
}

run_selected_setup() {
  if [[ "$DEPLOY_MODE" == "single" ]]; then
    run_setup
  elif [[ "$DUAL_PHASE" == "setup1" ]]; then
    run_setup_dual_setup1
  else
    run_setup_dual_setup2
  fi
}

run_selected_run() {
  if [[ "$DEPLOY_MODE" == "single" ]]; then
    run_run
  elif [[ "$DUAL_PHASE" == "setup1" ]]; then
    run_run_dual_setup1
  else
    run_run_dual_setup2
  fi
}

main() {
  ensure_python3
  select_deploy_target

  if [[ -z "$ACTION" ]]; then
    info "=== No flag provided — running --setup then --run ==="
    run_selected_setup
    run_selected_run
    return
  fi

  case "$ACTION" in
    --setup)   run_selected_setup ;;
    --run)     run_selected_run ;;
    --down)    do_down ;;
    --cleanup) do_cleanup ;;
  esac
}

main "$@"
