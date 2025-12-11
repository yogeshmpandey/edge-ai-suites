## Run Locally 

### TODOs

-[] Add support for GPU pipeline in DLSPS

-[] Add support for GPU Graphs using Qmassa

-[] Add support for multiple parallel pipeline trigger.

-[] Testing on different HWs.

-[] Update the DLSPS config so that it can take all the needed values as params. 

### 1) Prerequisites
- Docker and Docker Compose
- Host with enough CPU/GPU for the selected models

### 2) Configure environment
Update a `.env` in the repo root:

```
PROJECT_NAME=vlm-captioning
EVAM_HOST_PORT=8040
EVAM_PORT=8080
WHIP_SERVER_PORT=8889
DASHBOARD_PORT=4173
WEBRTC_PEER_ID=stream
HOST_IP=${HOST_IP}
MTX_WEBRTCICESERVERS2_0_USERNAME=localuser
MTX_WEBRTCICESERVERS2_0_PASSWORD=localpass
```

- If you need a proxy, set `http_proxy/https_proxy` as usual; they pass through to the containers.

### 3) Start the stack

```
docker compose --env-file .env up --build
```

Exposed host ports:
- 8040: REST API for pipelines
- 8889: WHIP/WebRTC signaling
- 4173: Metadata dashboard (reads `/tmp/results.jsonl`)

### 4) Provide a video source
The pipeline expects a reachable RTSP source. Use your camera’s RTSP URL or host a local test stream. 

### 5) Create the pipeline
Send a pipeline request  using the Web portal. 

### 6) View results
- Metadata file: `/tmp/results.jsonl` on the host
- Dashboard: `http://localhost:4173` (uses `WEBRTC_PEER_ID=stream` and `SIGNALING_URL=http://localhost:8889`)

### 7) Tear down

```
docker compose --env-file .env down
```
