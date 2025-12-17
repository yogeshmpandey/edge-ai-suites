# Live Video Captioning

GenAI-powered captioning for live video streams. Deploy the stack locally to ingest RTSP streams, generate real-time captions using Vision Language Models (VLMs), and view results in an interactive dashboard.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
- [GPU Monitoring](#gpu-monitoring-with-qmassa)

---

## Prerequisites

Before getting started, ensure you have:

- **Docker & Docker Compose** – installed and configured 
- **Compatible Hardware** – host with sufficient CPU/GPU resources for your chosen OpenVINO model
- **VLM Model** – OpenVINO-compatible Vision Language Model placed in `ov_models/`

---

## Quick Start

### Step 1: Configure Environment

Create a `.env` file in the repository root with the following variables:

```bash
PROJECT_NAME=vlm-captioning
EVAM_HOST_PORT=8040
EVAM_PORT=8080
WHIP_SERVER_PORT=8889
DASHBOARD_PORT=4173
WEBRTC_PEER_ID=stream
HOST_IP=<YOUR_HOST_IP>
MTX_WEBRTCICESERVERS2_0_USERNAME=<USERNAME>
MTX_WEBRTCICESERVERS2_0_PASSWORD=<PASSWORD>
```

> **Note:** Replace `<YOUR_HOST_IP>`, `<USERNAME>`, and `<PASSWORD>` with your actual values.

### Step 2: Download Models

Use the helper script to download and convert supported VLMs to OpenVINO format:

```bash
chmod +x download_models.sh
./download_models.sh [phi4|minicpm|gemma3|internvl2]
```

**Supported models:**
| Alias | Model |
|-------|-------|
| `phi4` | Phi-4-multimodal |
| `minicpm` | MiniCPM-V-2_6 |
| `gemma3` | Gemma-3-4b-it |
| `internvl2` | InternVL2-2B |

> Note you can download and use other OpenVINO supported VLM models 

Exported models are saved to `ov_models/<model>/` and automatically discovered by the dashboard.

> **For gated models** (e.g., MiniCPM-V-2_6): Set your Hugging Face token before running:
> ```bash
> export HF_TOKEN=<YOUR_HF_TOKEN>
> ```

### Step 3: Start Services

```bash
docker compose up --build
```

**Exposed ports:**
| Port | Service |
|------|---------|
| `8040` | REST API (pipeline management) |
| `8889` | WHIP/WebRTC signaling |
| `4173` | Dashboard UI |

### Step 4: Provide a Video Source

Connect a video source using one of these options:
- **RTSP camera** – use your camera's RTSP URL
- **Local test stream** – set up a local RTSP server for testing

### Step 5: Create the Pipeline

1. Open the dashboard at **http://localhost:4173**
2. Select your model from the dropdown
3. Configure and submit a pipeline request

### Step 6: View Results

- **Live Dashboard:** http://localhost:4173  
  (WebRTC stream with `PEER_ID=stream`, signaling at `localhost:8889`)
- **Raw Metadata:** `/tmp/results.jsonl` on the host

### Step 7: Stop Services

```bash
docker compose down
```

---

## Architecture

### DLStreamer Pipeline

The pipeline uses a tee element to split the video stream for parallel processing:

```
source → decode → tee
                   │
                   ├─→ [AI Branch] queue(leaky) → convert(BGRA) → rate(1fps) → gvagenai → queue(leaky) → metaconvert → metapublish → fakesink
                   │
                   └─→ [Preview Branch] queue(leaky) → convert(NV12) → rate(30fps) → appsink(drop=true)
```

| Branch | Purpose |
|--------|---------|
| **AI Branch** | Processes frames at 1 FPS through the VLM for caption generation |
| **Preview Branch** | Streams video at 30 FPS for real-time viewing |

---

## GPU Monitoring with Qmassa

The dashboard displays real-time GPU metrics using [qmassa](https://github.com/ulissesf/qmassa). Since GPU workloads run inside Docker containers, qmassa must run on the **host** with elevated privileges to monitor all GPU clients.

### Available Metrics

| Metric | Description |
|--------|-------------|
| **GPU Usage %** | Per-engine utilization (render, video, compute, etc.) |
| **VRAM** | Device memory usage (discrete GPUs only) |
| **Frequency** | Current and max GPU frequency (MHz) |
| **Power** | GPU and package power consumption (Watts) |

### Installation

Install Rust and qmassa on your host machine:

```bash
# 1. Install Rust (skip if already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# 2. Install qmassa
cargo install --locked qmassa
```

### Running Qmassa

#### Option A: Foreground (Interactive)

Start qmassa in a dedicated terminal:

```bash
sudo $(which qmassa) -x -t /tmp/qmassa-stats.json -m 1000
```

#### Option B: Background (Recommended for Production)

Run qmassa as a background service:

```bash
# Clean up stale files and start in background
sudo rm -f /tmp/qmassa.log /tmp/qmassa.pid
sudo bash -c 'nohup $(which qmassa) -x -t /tmp/qmassa-stats.json -m 1000 > /tmp/qmassa.log 2>&1 & echo $! > /tmp/qmassa.pid'
```

**Manage the background process:**

```bash
# Check status
pgrep -a qmassa

# View logs
tail -f /tmp/qmassa.log

# Stop qmassa
sudo pkill qmassa
```

### Command Reference

| Option | Description |
|--------|-------------|
| `-x` | Headless mode (no terminal UI) |
| `-t <file>` | Write JSON stats to specified file |
| `-m <ms>` | Update interval in milliseconds |

### Disabling GPU Monitoring

To disable GPU monitoring entirely, add this to your `.env` file:

```bash
QMASSA_ENABLED=false
```