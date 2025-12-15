## Live Video Captioning - Getting Started

GenAI-powered captioning for live video streams. Deploy the stack locally to ingest RTSP, generate captions, and view results in the dashboard.

### Prerequisites
- Docker and Docker Compose installed (non-root Docker recommended)
- Host with sufficient CPU/GPU for your chosen OpenVINO model
- OpenVINO-compatible VLM model in `ov_models`

### Quick Start
1) Configure environment: create `.env` in the repo root
```
PROJECT_NAME=vlm-captioning
EVAM_HOST_PORT=8040
EVAM_PORT=8080
WHIP_SERVER_PORT=8889
DASHBOARD_PORT=4173
WEBRTC_PEER_ID=stream
HOST_IP=<HOST_IP>
MTX_WEBRTCICESERVERS2_0_USERNAME=<UserName>
MTX_WEBRTCICESERVERS2_0_PASSWORD=<Password>
```

2) (Optional) Download/convert OpenVINO models into `ov_models`

Use the helper script to create `.venv`, install export dependencies (from OpenVINO GenAI 2025.4), and export one of the supported VLMs (Phi-4-multimodal, MiniCPM-V-2_6 int4, Gemma-3-4b-it, InternVL2-2B, SmolVLM2-256M-Video-Instruct) or any other Hugging Face repo id (warned):

```
chmod +x download_models.sh
./download_models.sh [phi4|minicpm|gemma3|internvl2|smolvlm2|<hf_repo_id>]
```

Exports land under `ov_models/<model>`.

For gated models like MiniCPM-V-2_6, set `HF_TOKEN` (or `HUGGINGFACEHUB_API_TOKEN`) before running the script.

Models placed in `ov_models/` are auto-discovered by the dashboard API and shown as a dropdown for `Model Name`.

3) Start services
```
docker compose --env-file .env up --build
```
Exposed host ports: 8040 (REST pipelines), 8889 (WHIP/WebRTC signaling), 4173 (dashboard)

3) Provide a video source
- Use a reachable RTSP stream (camera RTSP URL or local test feed)

4) Create the pipeline
- Submit a pipeline request via the web portal

5) View results
- Dashboard: http://localhost:4173 (uses `WEBRTC_PEER_ID=stream`, `SIGNALING_URL=http://localhost:8889`)
- Metadata: `/tmp/results.jsonl` on the host

6) Stop services
```
docker compose down
```

### TODOs
- [ ] Add support for GPU graphs using Qmassa
- [ ] Test on additional hardware targets
- [ ] Update the DLSPS config to take all required values as parameters
