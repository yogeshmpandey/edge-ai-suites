# Get Started

This guide walks you through running Live Video Captioning with Docker Compose.

## Prerequisites

- Docker Engine + Docker Compose
- RTSP stream source (live camera or test feed)
- OpenVINO-compatible VLM in `ov_models/`

See [System Requirements](./system-requirements.md) for details.

## 1. Configure environment

Create a `.env` file in the repository root and update the HOST_IP with the system IP.

```bash
WHIP_SERVER_IP=mediamtx
WHIP_SERVER_PORT=8889
WHIP_SERVER_TIMEOUT=30s
PROJECT_NAME=live-captioning
HOST_IP=<HOST_IP>
EVAM_HOST_PORT=8040
EVAM_PORT=8080
DASHBOARD_PORT=4173
WEBRTC_PEER_ID=stream
METADATA_POLL_SECONDS=0.5
AGENT_MODE=False
```


## 2. Download/export models

```bash
chmod +x download_models.sh
./download_models.sh [internvl2_1B|internvl2_2B|gemma3]
```

For other OpenVINO supported models, provide the HuggingFace model name.

```
./download_models.sh OpenGVLab/InternVL2_5-1B
```

For gated models export the huggingface token:

```bash
export HF_TOKEN=<YOUR_HUGGING_FACE_TOKEN>
```

## 3. Start the stack

```bash
docker compose up --build
```

Exposed services (defaults):
- Pipeline API: `http://<HOST_IP>:8040`
- WebRTC signaling: `ws://<HOST_IP>:8889`
- Dashboard UI: `http://<HOST_IP>:4173`

## 4. Run a captioning pipeline

1. Open the dashboard at `http://<HOST_IP>:4173`.
2. Enter an RTSP URL.
3. Select a VLM model.
4. Edit prompt/max tokens as needed.
5. Click **Start**.

## 5. Stop services

```bash
docker compose down
```

## Next

- [API Reference](./api-reference.md)
- [Known Issues](./known-issues.md)
