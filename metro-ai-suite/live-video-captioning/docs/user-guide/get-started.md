# Get Started

This guide walks you through running Live Video Captioning with Docker Compose.

## Prerequisites

- Docker Engine + Docker Compose
- RTSP stream source (live camera or test feed)
- OpenVINO-compatible VLM in `ov_models/`

See [System Requirements](./system-requirements.md) for details.

## 1) Configure environment

Create a `.env` file in the repository root:

```bash
PROJECT_NAME=live-video-captioning
EVAM_HOST_PORT=8040
EVAM_PORT=8080
WHIP_SERVER_PORT=8889
DASHBOARD_PORT=4173
WEBRTC_PEER_ID=stream
HOST_IP=<YOUR_HOST_IP>
MTX_WEBRTCICESERVERS2_0_USERNAME=<TURN_USER>
MTX_WEBRTCICESERVERS2_0_PASSWORD=<TURN_PASS>
AGENT_MODE=false
METADATA_POLL_SECONDS=1
PIPELINE_NAME=GenAI_Pipeline_on_CPU
```

Notes:
- `HOST_IP` must be reachable by the browser client for WebRTC signaling.
- `PIPELINE_SERVER_URL` defaults to `http://video-ingestion:8080`.

## 2) (Optional) Download/export models

```bash
chmod +x download_models.sh
./download_models.sh [phi4|minicpm|gemma3|internvl2]
```

For gated models (for example, MiniCPM-V-2_6):

```bash
export HF_TOKEN=<YOUR_HUGGING_FACE_TOKEN>
```

## 3) Start the stack

```bash
docker compose up --build
```

Exposed services (defaults):
- Pipeline API: `http://<HOST_IP>:8040`
- WebRTC signaling: `ws://<HOST_IP>:8889`
- Dashboard UI: `http://<HOST_IP>:4173`

## 4) Run a captioning pipeline

1. Open the dashboard at `http://<HOST_IP>:4173`.
2. Enter an RTSP URL.
3. Select a VLM model.
4. Edit prompt/max tokens as needed.
5. Click **Start**.

## 5) Stop services

```bash
docker compose down
```

## Next

- [API Reference](./api-reference.md)
- [Known Issues](./known-issues.md)
