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

### DLStreamer Queue Configuration

source → decode → tee
                   ├─→ queue(leaky) → convert(BGRA) → rate(1fps) → gvagenai → queue(leaky) → metaconvert → metapublish → fakesink
                   └─→ queue(leaky) → convert(NV12) → rate(30fps) → appsink(drop=true)

                   

2) (Optional) Download/convert OpenVINO models into `ov_models`

Use the helper script to create `.venv`, install export dependencies (from OpenVINO GenAI 2025.4), and export one of the supported VLMs (Phi-4-multimodal, MiniCPM-V-2_6, Gemma-3-4b-it, InternVL2-2B) or any other Hugging Face repo id (warned):

```
chmod +x download_models.sh
./download_models.sh [phi4|minicpm|gemma3|internvl2|smolvlm2]
```

Exports land under `ov_models/<model>`.

For gated models like MiniCPM-V-2_6, set `export HF_TOKEN=<HF_TOKEN>` (or `HUGGINGFACEHUB_API_TOKEN`) before running the script.

Models placed in `ov_models/` are auto-discovered by the dashboard API and shown as a dropdown for `Model Name`.

3) Start services
```
docker compose up --build
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

### GPU Monitoring with Qmassa

The dashboard displays real-time GPU metrics (utilization, memory, frequency, power) using [qmassa](https://github.com/ulissesf/qmassa). Since GPU workloads run inside Docker containers, qmassa must run on the **host** with elevated privileges to see all GPU clients.

#### Install qmassa on host

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# Install qmassa
cargo install --locked qmassa
```

#### Run qmassa on host

On a new terminal, start qmassa in headless mode with sudo to capture GPU metrics from all processes:

```bash
sudo $(which qmassa) -x -t /tmp/qmassa-stats.json -m 1000
```

Options:
- `-x` : Headless mode (no TUI)
- `-t /tmp/qmassa-stats.json` : Write stats to JSON file
- `-m 1000` : Update interval in milliseconds (1 second)

The dashboard container reads from `/tmp/qmassa-stats.json` (mounted read-only) and displays:
- **GPU Usage %** : Per-engine utilization (render, video, compute, etc.)
- **VRAM** : Device memory usage (discrete GPUs)
- **Frequency** : Current and max GPU frequency (MHz)
- **Power** : GPU and package power consumption (Watts)

#### Run qmassa in background

To run qmassa as a background service:

```bash
# Start in background (remove stale files first to avoid permission issues)
sudo rm -f /tmp/qmassa.log /tmp/qmassa.pid
sudo bash -c "nohup $(which qmassa) -x -t /tmp/qmassa-stats.json -m 1000 > /tmp/qmassa.log 2>&1 & echo \$! > /tmp/qmassa.pid"

# Check if running
pgrep -a qmassa

# Stop qmassa
sudo pkill qmassa
```

#### Disable GPU monitoring

To disable GPU monitoring, set in `.env`:
```
QMASSA_ENABLED=false
```.