#!/bin/bash

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

export REGISTRY_URL=${REGISTRY_URL:-}
export PROJECT_NAME=${PROJECT_NAME:-}
export TAG=${TAG:-latest}
export RTSP_STREAM_PORT=${RTSP_STREAM_PORT:-8554}
export WATCH_BATCH_SIZE=${WATCH_BATCH_SIZE:-10}
export BATCH_JOB_POLL_INTERVAL_SECONDS=${BATCH_JOB_POLL_INTERVAL_SECONDS:-0.5}
export BATCH_JOB_TIMEOUT_SECONDS=${BATCH_JOB_TIMEOUT_SECONDS:-3600}
RTSP_STREAM_BIND_IP=${RTSP_STREAM_BIND_IP:-0.0.0.0}


[[ -n "$REGISTRY_URL" ]] && REGISTRY_URL="${REGISTRY_URL%/}/"
[[ -n "$PROJECT_NAME" ]] && PROJECT_NAME="${PROJECT_NAME%/}/"
REGISTRY="${REGISTRY_URL}${PROJECT_NAME}"

export REGISTRY="${REGISTRY:-}"

# Display info about the registry being used
if [ -z "$REGISTRY" ]; then
  echo -e "${YELLOW}Warning: No registry prefix set. Images will be tagged without a registry prefix.${NC}"
  echo "Using local image names with tag: ${TAG}"
else
  echo "Using registry prefix: ${REGISTRY}"
fi


# Helper functions for colored output
print_error() {
    echo -e "${RED}Error: $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}Warning: $1${NC}"
}

print_success() {
    echo -e "${GREEN}Success: $1${NC}"
}

print_info() {
    echo -e "${BLUE}Info: $1${NC}"
}

print_header() {
    echo -e "${PURPLE}=== $1 ===${NC}"
}

MQTT_SECRETS_FILE="./resources/mqtt-secrets"
INTERSECTIONS_CONFIG_PATH="${INTERSECTIONS_CONFIG_PATH:-./resources/broker-config/intersections.yaml}"
INTERSECTIONS_PARSER="./scripts/parse-intersections.sh"
FRIGATE_CONFIG_FILE="${FRIGATE_CONFIG_FILE:-./resources/frigate-config/config.yml}"

# Parsed intersections config (populated by load_intersections)
INTERSECTION_IDS=()
INTERSECTION_NAMES=()
INTERSECTION_IPS=()
INTERSECTION_ENABLED=()
INTERSECTION_CAMERAS=()   # records of "id|camera_name|rtsp_url"
INTERSECTIONS_PARSE_ERROR=""

resolve_mqtt_credentials() {
    if [[ -n "${MQTT_USER}" && -n "${MQTT_PASSWORD}" ]]; then
        print_info "Using provided MQTT credentials (MQTT_USER=${MQTT_USER})"
        return 0
    fi

    if ! bash "$(dirname "${BASH_SOURCE[0]}")/scripts/gen-mqtt-secrets.sh"; then
        return 1
    fi

    # shellcheck source=/dev/null
    source "${MQTT_SECRETS_FILE}"
    export MQTT_USER MQTT_PASSWORD
    print_info "MQTT credentials loaded from ${MQTT_SECRETS_FILE}"
}

# Get the host IP address
get_host_ip() {
    # Try different methods to get the host IP
    if command -v ip &> /dev/null; then
        # Use ip command if available (Linux)
        HOST_IP=$(ip route get 1 | sed -n 's/^.*src \([0-9.]*\) .*$/\1/p')
    elif command -v ifconfig &> /dev/null; then
        # Use ifconfig if available (Linux/Mac)
        HOST_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | head -n 1)
    else
        # Fallback to hostname command
        HOST_IP=$(hostname -I | awk '{print $1}')
    fi

    # Fallback to localhost if we couldn't determine the IP
    if [ -z "$HOST_IP" ]; then
        HOST_IP="localhost"
        print_warning "Could not determine host IP, using localhost instead."
    fi

    echo "$HOST_IP"
}

# ─── Intersections config: single source of RTSP/MQTT configuration ──────

# Parse intersections.yaml into the INTERSECTION_* arrays.
# Returns: 0 entries found, 1 unreadable file, 2 no entries, 3 invalid entry.
load_intersections() {
    INTERSECTION_IDS=()
    INTERSECTION_NAMES=()
    INTERSECTION_IPS=()
    INTERSECTION_ENABLED=()
    INTERSECTION_CAMERAS=()

    if [ ! -f "${INTERSECTIONS_PARSER}" ]; then
        print_error "Missing parser script: ${INTERSECTIONS_PARSER}"
        return 1
    fi

    local records rc errors_file
    errors_file="$(mktemp)"
    # Use >| to force the redirect even if the caller's shell has `noclobber`
    # set (setup.sh is designed to be sourced, so it inherits shell options).
    records="$(bash "${INTERSECTIONS_PARSER}" "${INTERSECTIONS_CONFIG_PATH}" "${RTSP_STREAM_PORT}" 2>|"${errors_file}")"
    rc=$?
    INTERSECTIONS_PARSE_ERROR="$(cat "${errors_file}")"
    rm -f "${errors_file}"
    [ ${rc} -ne 0 ] && return ${rc}

    local kind field2 field3 field4 field5 field6
    while IFS='|' read -r kind field2 field3 field4 field5 field6; do
        case "${kind}" in
            I)
                INTERSECTION_IDS+=("${field2}")
                INTERSECTION_NAMES+=("${field3}")
                INTERSECTION_IPS+=("${field4}")
                : "${field5}" # mqtt_port; not tracked separately by setup.sh
                INTERSECTION_ENABLED+=("${field6}")
                ;;
            C)
                INTERSECTION_CAMERAS+=("${field2}|${field3}|${field4}")
                ;;
        esac
    done <<< "${records}"

    [ ${#INTERSECTION_IDS[@]} -eq 0 ] && return 2
    return 0
}

# Count cameras belonging to an intersection id
count_cameras_for() {
    local target="$1" count=0 record
    for record in "${INTERSECTION_CAMERAS[@]}"; do
        [ "${record%%|*}" = "${target}" ] && count=$((count + 1))
    done
    echo "${count}"
}

print_intersections_summary() {
    local idx
    for idx in "${!INTERSECTION_IDS[@]}"; do
        local state=""
        [ "${INTERSECTION_ENABLED[idx]}" = "false" ] && state=" ${YELLOW}(disabled)${NC}"
        echo -e "  ${WHITE}$((idx + 1)).${NC} ${CYAN}${INTERSECTION_IDS[idx]}${NC} - ${INTERSECTION_NAMES[idx]} @ ${INTERSECTION_IPS[idx]} ($(count_cameras_for "${INTERSECTION_IDS[idx]}") cameras)${state}"
    done
}

print_intersections_help() {
    print_info "Configure your intersections in: ${CYAN}${INTERSECTIONS_CONFIG_PATH}${NC}"
    cat <<'EXAMPLE'

  intersections:
    - id: si1                       # must match Frigate camera prefix si1-camera*
      name: Main Street and 1st Ave
      ip: 10.0.0.11                 # MQTT broker for this intersection
      cameras:
        - name: si1-camera1
          url: rtsp://10.0.0.21:8554/camera1   # each camera is a self-contained RTSP URL
        - name: si1-camera2
          url: rtsp://10.0.0.22:8554/camera1
        - name: si1-camera3
          url: rtsp://10.0.0.23:8554/camera1
        - name: si1-camera4
          url: rtsp://10.0.0.24:8554/camera1

EXAMPLE
    print_info "Update the file and re-run this command."
}

# Write a single local intersection (single-node demo) to the config file
seed_local_intersection() {
    local ip="$1"
    local config_dir
    config_dir="$(dirname "${INTERSECTIONS_CONFIG_PATH}")"
    mkdir -p "${config_dir}" || return 1

    {
        echo "# Smart NVR intersections - single source of configuration."
        echo "# Generated by setup.sh for a single-node deployment. Edit as needed."
        echo "intersections:"
        echo "  - id: si1"
        echo "    name: Smart Intersection 1"
        echo "    ip: ${ip}"
        echo "    cameras:"
        local cam_num
        for cam_num in 1 2 3 4; do
            echo "      - name: si1-camera${cam_num}"
            echo "        url: rtsp://${ip}:${RTSP_STREAM_PORT}/camera${cam_num}"
        done
    } > "${INTERSECTIONS_CONFIG_PATH}" || return 1

    print_success "Created ${INTERSECTIONS_CONFIG_PATH} with si1 @ ${ip} (cameras 1-4)"
}

# Ask whether the preconfigured intersections should be used
confirm_intersections() {
    local count=${#INTERSECTION_IDS[@]}
    print_info "Found ${count} preconfigured intersection(s) in ${INTERSECTIONS_CONFIG_PATH}:"
    print_intersections_summary

    case "${INTERSECTIONS_AUTO_CONFIRM:-}" in
        [Tt]rue|1|[Yy]|[Yy]es|[Yy]ES|TRUE|YES)
            print_info "INTERSECTIONS_AUTO_CONFIRM is set - using the preconfigured intersections."
            return 0
            ;;
    esac

    if [ ! -t 0 ]; then
        print_info "Non-interactive shell - using the preconfigured intersections."
        return 0
    fi

    local answer
    read -r -p "$(echo -e "${BLUE}Would you like to use them? (Y/N) [Y]: ${NC}")" answer
    case "${answer:-Y}" in
        [Yy]|[Yy][Ee][Ss]) return 0 ;;
        *)
            print_warning "Setup cancelled."
            print_intersections_help
            return 1
            ;;
    esac
}

# Resolve the intersections config, seeding a local one for single-node runs
resolve_intersections() {
    local allow_seed="$1"
    load_intersections
    local rc=$?

    case ${rc} in
        0) confirm_intersections; return $? ;;
        3)
            print_error "Invalid intersections config: ${INTERSECTIONS_PARSE_ERROR:-see ${INTERSECTIONS_CONFIG_PATH}}"
            print_intersections_help
            return 1
            ;;
        1|2)
            if [ "${allow_seed}" = "true" ]; then
                print_info "No intersections configured - creating one for this machine."
                seed_local_intersection "$(get_host_ip)" || return 1
                load_intersections || return 1
                print_intersections_summary
                return 0
            fi
            print_error "No intersections configured in ${INTERSECTIONS_CONFIG_PATH}"
            print_intersections_help
            return 1
            ;;
    esac
    return 1
}

# Generate the Frigate camera blocks from the intersections config
generate_scenescape_config() {
    local config_file="${FRIGATE_CONFIG_FILE}"

    cp "./resources/frigate-config/config-scenescape.yml" "${config_file}"
    printf '\ncameras:\n' >> "${config_file}"

    local idx
    for idx in "${!INTERSECTION_IDS[@]}"; do
        local si_id="${INTERSECTION_IDS[idx]}"

        if [ "${INTERSECTION_ENABLED[idx]}" = "false" ]; then
            print_warning "Skipping disabled intersection ${si_id}"
            continue
        fi

        local added=0 record
        for record in "${INTERSECTION_CAMERAS[@]}"; do
            IFS='|' read -r cam_owner cam_name cam_url <<< "${record}"
            [ "${cam_owner}" != "${si_id}" ] && continue

            cat >> "${config_file}" <<CAMERA_BLOCK

  ${cam_name}:
    ffmpeg:
      inputs:
        - path: ${cam_url}
          input_args: preset-rtsp-generic
          roles:
            - record
      output_args:
        record: -f segment -segment_time 10 -segment_format mp4 -reset_timestamps 1 -strftime 1 -c:v copy -movflags +faststart
    detect:
      enabled: false
    motion:
      enabled: false
    snapshots:
      enabled: false
    record:
      enabled: true
      retain:
        days: 1
        mode: all
CAMERA_BLOCK
            added=$((added + 1))
        done

        print_success "Added ${si_id} (${INTERSECTION_NAMES[idx]}) with ${added} camera(s)"
    done

    printf '\nversion: 0.15-1\n' >> "${config_file}"
}

configure_scenescape_setup() {

    if [ "${NVR_SCENESCAPE}" = "True" ] || [ "${NVR_SCENESCAPE}" = "true" ]; then
        print_info "NVR_SCENESCAPE is enabled - configuring Scenescape mode"

        local metro_recipe_dir
        metro_recipe_dir="$(cd .. && pwd)/metro-vision-ai-app-recipe"
        local rtsp_ip="${SI_RTSP_HOST:-$(get_host_ip)}"

        if [ "${SCENESCAPE_NVR_ONLY}" != "true" ]; then
            # Configure SI stack: compose + DL Streamer
            local dlstreamer_config="${metro_recipe_dir}/smart-intersection/src/dlstreamer-pipeline-server/config.json"
            cp "./resources/compose-scenescape-rtsp.yml" "${metro_recipe_dir}/compose-scenescape.yml"
            cp "./resources/si-rtsp-config.json" "${dlstreamer_config}"
            sed -i "s/{RTSP_STREAM_IP}/${rtsp_ip}/g" "${dlstreamer_config}"
            sed -i "s/{RTSP_STREAM_PORT}/${RTSP_STREAM_PORT}/g" "${dlstreamer_config}"
        fi

        if [ "${SCENESCAPE_SI_ONLY}" != "true" ]; then
            # NVR side: intersections.yaml drives the Frigate camera configuration
            local allow_seed="true"
            [ "${SCENESCAPE_NVR_ONLY:-}" = "true" ] && allow_seed="false"
            if ! resolve_intersections "${allow_seed}"; then
                return 1
            fi
            if ! generate_scenescape_config; then
                print_error "Failed to generate the Frigate configuration."
                return 1
            fi
        fi

        print_success "Scenescape configuration activated"
    else
        print_info "NVR_SCENESCAPE is disabled - using default configuration"
        cp "./resources/frigate-config/config-default.yml" "${FRIGATE_CONFIG_FILE}"
        print_success "Default Frigate configuration activated"
    fi
}


download_videos() {
    local video_dir="./resources/videos"
    local video_url="https://github.com/open-edge-platform/edge-ai-resources/raw/refs/heads/main/videos"
    local videos=(1122north_h264.ts 1122east_h264.ts 1122south_h264.ts 1122west_h264.ts)
    mkdir -p "$video_dir"
    local downloaded=false
    for video in "${videos[@]}"; do
        if [ ! -f "${video_dir}/${video}" ]; then
            print_info "Downloading ${video}..."
            if ! curl -fL "${video_url}/${video}" -o "${video_dir}/${video}"; then
                print_error "Failed to download ${video}"
                return 1
            fi
            downloaded=true
        fi
    done
    [[ "$downloaded" == true ]] && print_success "Demo videos downloaded" || print_info "Demo videos already present"
}

start_rtsp_streamer() {
    local videos=(1122north_h264.ts 1122east_h264.ts 1122south_h264.ts 1122west_h264.ts)
    for video in "${videos[@]}"; do
        if [ ! -f "./resources/videos/${video}" ]; then
            print_error "Missing video: ./resources/videos/${video}"
            return 1
        fi
    done
    print_info "Starting MediaMTX RTSP streamer on ${RTSP_STREAM_BIND_IP}:${RTSP_STREAM_PORT}"
    RTSP_STREAM_BIND_IP="$RTSP_STREAM_BIND_IP" RTSP_STREAM_PORT="$RTSP_STREAM_PORT" \
        docker compose -p smartnvr-mediamtx -f streamer/docker-compose.yml up -d
}

stop_rtsp_streamer() {
    if [ -f "streamer/docker-compose.yml" ]; then
        docker compose -p smartnvr-mediamtx -f streamer/docker-compose.yml down || true
    fi
}

start_scenescape() {
    local metro_recipe_dir
    metro_recipe_dir="$(cd .. && pwd)/metro-vision-ai-app-recipe"
    if [ ! -f "${metro_recipe_dir}/compose-scenescape.yml" ]; then
        print_error "Smart Intersection compose not found at ${metro_recipe_dir}"
        return 1
    fi
    if [ ! -f "${metro_recipe_dir}/smart-intersection/src/secrets/supass" ]; then
        (cd "${metro_recipe_dir}" && bash install.sh smart-intersection)
    fi
    docker compose -f "${metro_recipe_dir}/compose-scenescape.yml" --env-file "${metro_recipe_dir}/.env" up -d
}

stop_scenescape() {
    local metro_recipe_dir
    metro_recipe_dir="$(cd .. && pwd)/metro-vision-ai-app-recipe"
    if [ -f "${metro_recipe_dir}/compose-scenescape.yml" ]; then
        docker compose -f "${metro_recipe_dir}/compose-scenescape.yml" --env-file "${metro_recipe_dir}/.env" down || true
    fi
}

validate_environment() {
    export NVR_SCENESCAPE="${NVR_SCENESCAPE:-false}"

    # Check for VSS endpoint — one nginx proxy serves both summary and search
    if [ -z "${VSS_IP}" ]; then
        print_error "VSS_IP environment variable is required"
        print_info "Please set it to the IP address of your Video Search and Summarization (VSS) service"
        return 1
    fi
    export VSS_PORT="${VSS_PORT:-12345}"
    print_info "Using VSS endpoint: ${VSS_IP}:${VSS_PORT}"

    # Resolve MQTT credentials — auto-generates if not provided by the user
    if ! resolve_mqtt_credentials; then
        print_error "Could not resolve MQTT credentials. Aborting."
        return 1
    fi
}

# Function to start the services
start_services() {
    print_header "Starting NVR Event Router Services"
    HOST_IP=$(get_host_ip)
    export HOST_IP
    # Validate environment variables and exit if validation fails
    if ! validate_environment; then
        print_error "Environment validation failed. Please set the required variables."
        return 1
    fi

    if ! configure_scenescape_setup; then
        return 1
    fi

    if [ "${NVR_SCENESCAPE}" = "True" ] || [ "${NVR_SCENESCAPE}" = "true" ]; then
        if ! download_videos; then
            return 1
        fi
        if ! start_rtsp_streamer; then
            return 1
        fi
        if ! start_scenescape; then
            return 1
        fi
    fi

    print_info "Starting Docker Compose services..."
    docker compose -f docker/compose.yaml up -d
    if [ $? -eq 0 ]; then
    sleep 5
    if [ "${NVR_SCENESCAPE}" = "True" ] || [ "${NVR_SCENESCAPE}" = "true" ]; then
        docker network connect metro-vision-ai-app-recipe_scenescape nvr-event-router 2>/dev/null || true
    fi
    sleep 5
    print_success "Services are starting up..."
    print_info "UI will be available at: ${CYAN}http://${HOST_IP}:7860${NC}"
    else
        print_error "Docker Compose failed to start services."
        return 1
    fi
}

# Function to stop the services
stop_services() {
    print_header "Stopping NVR Event Router Services"
    print_info "Stopping NVR Event Router services..."
    docker compose -f docker/compose.yaml down
    stop_scenescape
    stop_rtsp_streamer
    print_success "All services stopped."
}

# ─── Remote mode: distributed node deployment ────────────────────────────

start_si_services() {
    print_header "Starting SI (System 1 / SI-only mode)"
    if [ "${NVR_SCENESCAPE}" != "True" ] && [ "${NVR_SCENESCAPE}" != "true" ]; then
        print_error "start-si requires NVR_SCENESCAPE=true"
        print_info "Run: export NVR_SCENESCAPE=true"
        return 1
    fi
    HOST_IP=$(get_host_ip)
    export HOST_IP

    # Start local RTSP streamer only when no external stream source is provided and not already running
    local rtsp_host="${SI_RTSP_HOST:-}"
    if [ -z "${rtsp_host}" ] || [ "${rtsp_host}" = "${HOST_IP}" ] || [ "${rtsp_host}" = "localhost" ]; then
        if docker ps --filter "name=^mediamtx$" --filter "status=running" --format '{{.Names}}' | grep -q .; then
            print_info "Local RTSP streamer already running - skipping"
        else
            print_info "No external RTSP source set - starting local MediaMTX streamer"
            if ! download_videos; then
                return 1
            fi
            if ! start_rtsp_streamer; then
                return 1
            fi
        fi
    else
        print_info "External RTSP source detected (${rtsp_host}) - skipping local streamer"
    fi

    if ! SCENESCAPE_SI_ONLY=true configure_scenescape_setup; then
        return 1
    fi

    if ! start_scenescape; then
        return 1
    fi

    local nvr_rtsp_host="${rtsp_host:-${HOST_IP}}"
    print_success "SI services are running on System 1."
    echo ""
    print_info "System 1 IP: ${CYAN}${HOST_IP}${NC}"
    print_info "SI RTSP: ${CYAN}${nvr_rtsp_host}:${RTSP_STREAM_PORT}${NC}  |  SI MQTT: ${CYAN}${HOST_IP}:1883${NC}"
    echo ""
    print_info "On System 2 (SmartNVR machine), add this intersection to ${CYAN}resources/broker-config/intersections.yaml${NC}:"
    cat <<EXAMPLE

  intersections:
    - id: si1
      name: Smart Intersection 1
      ip: ${HOST_IP}
      cameras:
        - name: si1-camera1
          ip: ${nvr_rtsp_host}
        - name: si1-camera2
          ip: ${nvr_rtsp_host}
        - name: si1-camera3
          ip: ${nvr_rtsp_host}
        - name: si1-camera4
          ip: ${nvr_rtsp_host}

EXAMPLE
    print_info "Then run:"
    echo -e "  ${CYAN}export NVR_SCENESCAPE=true${NC}"
    echo -e "  ${CYAN}export VSS_IP=<vss_ip>${NC}"
    echo -e "  ${CYAN}export VSS_PORT=<vss_port>   # optional, default 12345${NC}"
    echo -e "  ${CYAN}source setup.sh start-nvr${NC}"
    echo -e "  ${CYAN}# Optional: export RTSP_STREAM_PORT=<port>   # default ${RTSP_STREAM_PORT}${NC}"
}

stop_si_services() {
    print_header "Stopping SI (System 1)"
    stop_scenescape
    if docker ps --filter "name=^mediamtx$" --filter "status=running" --format '{{.Names}}' | grep -q .; then
        read -r -p "Local RTSP streamer is running. Stop it too? [y/N] " answer
        if [[ "${answer}" =~ ^[Yy]$ ]]; then
            stop_rtsp_streamer
            print_success "SI and RTSP streamer stopped."
        else
            print_info "RTSP streamer left running. Stop manually with: source setup.sh stop-streamer"
            print_success "SI services stopped."
        fi
    else
        print_success "SI services stopped."
    fi
}

start_nvr_services() {
    print_header "Starting SmartNVR (System 2 / NVR-only mode)"
    if [ "${NVR_SCENESCAPE}" != "True" ] && [ "${NVR_SCENESCAPE}" != "true" ]; then
        print_error "start-nvr requires NVR_SCENESCAPE=true"
        print_info "Run: export NVR_SCENESCAPE=true"
        return 1
    fi
    HOST_IP=$(get_host_ip)
    export HOST_IP

    if ! validate_environment; then
        print_error "Environment validation failed. Please set the required variables."
        return 1
    fi

    if ! SCENESCAPE_NVR_ONLY=true configure_scenescape_setup; then
        return 1
    fi

    print_info "Starting Docker Compose services..."
    docker compose -f docker/compose.yaml up -d
    if [ $? -eq 0 ]; then
        sleep 5
        print_success "SmartNVR services are starting up..."
        print_info "UI will be available at: ${CYAN}http://${HOST_IP}:7860${NC}"
        if [ -n "${SCENESCAPE_MQTT_BROKER}" ]; then
            print_info "MQTT broker seeded from env: ${SCENESCAPE_MQTT_BROKER}"
        else
            print_info "MQTT brokers loaded from ${INTERSECTIONS_CONFIG_PATH} (one per intersection)."
        fi
    else
        print_error "Docker Compose failed to start services."
        return 1
    fi
}

stop_nvr_services() {
    print_header "Stopping SmartNVR (System 2)"
    docker compose -f docker/compose.yaml down
    print_success "SmartNVR services stopped."
}

# Function to display help
show_help() {
    print_header "NVR Event Router Setup Script"
    echo -e "${WHITE}Usage:${NC} $0 [command]"
    echo ""
    echo -e "${WHITE}Commands:${NC}"
    echo -e "  ${GREEN}start${NC}          - Single-node: start everything (RTSP + SI + Frigate + event router)"
    echo -e "  ${RED}stop${NC}           - Single-node: stop everything"
    echo -e "  ${YELLOW}restart${NC}        - Single-node: restart everything"
    echo -e "  ${GREEN}start-streamer${NC} - RTSP-only: start MediaMTX streamer "
    echo -e "  ${RED}stop-streamer${NC}  - RTSP-only: stop MediaMTX streamer"
  echo -e "  ${GREEN}start-si${NC}       - Distributed Node System 1: start SI services (starts local RTSP streamer unless SI_RTSP_HOST is set)"
  echo -e "  ${RED}stop-si${NC}        - Distributed Node System 1: stop SI services (prompts to stop local RTSP streamer if running)"
  echo -e "  ${GREEN}start-nvr${NC}      - Distributed Node System 2: start SmartNVR only (intersections come from intersections.yaml)"
  echo -e "  ${RED}stop-nvr${NC}       - Distributed Node System 2: stop SmartNVR"
    echo -e "  ${BLUE}help${NC}           - Display this help message"
    echo ""
    echo -e "${WHITE}Intersections:${NC}"
    echo -e "  All RTSP/MQTT endpoints come from ${CYAN}${INTERSECTIONS_CONFIG_PATH}${NC}."
    echo -e "  Add one entry per intersection (name, ip and its 4 cameras) before starting."
    echo -e "  Set ${CYAN}INTERSECTIONS_AUTO_CONFIRM=true${NC} to skip the confirmation prompt."
    echo ""
    echo -e "${WHITE}Examples:${NC}"
    echo -e "  ${CYAN}source setup.sh start${NC}          # Single-node: start all services"
    echo -e "  ${CYAN}source setup.sh stop${NC}           # Single-node: stop all services"
    echo -e "  ${CYAN}source setup.sh restart${NC}        # Single-node: restart all services"
    echo -e "  ${CYAN}source setup.sh start-streamer${NC} # RTSP-only: start MediaMTX streamer"
    echo ""
    echo -e "  # Distributed Node — System 1 (SI + RTSP):${NC}"
    echo -e "  ${CYAN}export NVR_SCENESCAPE=true${NC}"
    echo -e "  ${CYAN}source setup.sh start-si${NC}"
    echo ""
    echo -e "  # Distributed Node — System 2 (SmartNVR):${NC}"
    echo -e "  ${CYAN}export NVR_SCENESCAPE=true${NC}"
    echo -e "  ${CYAN}export VSS_IP=<ip>   # VSS_PORT optional, default 12345${NC}"
    echo -e "  ${CYAN}# edit resources/broker-config/intersections.yaml with your intersections${NC}"
    echo -e "  ${CYAN}source setup.sh start-nvr${NC}"
    echo ""
}

# Main script logic
case "$1" in
    start-streamer)
        print_header "Starting RTSP Streamer"
        HOST_IP=$(get_host_ip)
        export HOST_IP
        if ! download_videos; then
            exit 1
        fi
        if ! start_rtsp_streamer; then
            exit 1
        fi
        print_success "RTSP streamer running on ${HOST_IP}:${RTSP_STREAM_PORT}"
        ;;
    stop-streamer)
        print_header "Stopping RTSP Streamer"
        stop_rtsp_streamer
        print_success "RTSP streamer stopped."
        ;;
    start)
        start_services
        ;;
    stop)
        stop_services
        ;;
    restart)
        print_header "Restarting NVR Event Router Services"
        stop_services
        sleep 5
        start_services
        ;;
    start-si)
        start_si_services
        ;;
    stop-si)
        stop_si_services
        ;;
    start-nvr)
        start_nvr_services
        ;;
    stop-nvr)
        stop_nvr_services
        ;;
    help|-h|--help)
        show_help
        ;;
    *)
        # Default behavior - show help
        show_help
        ;;
esac
