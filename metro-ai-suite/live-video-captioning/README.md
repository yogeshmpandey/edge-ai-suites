# Live Video Captioning

Deploy AI-powered captioning for live video streams with Intel DLStreamer and OpenVINO Vision Language Models. Process RTSP streams, generate real-time captions, and monitor performance metrics on an intuitive dashboard.

## Overview

Live Video Captioning simplifies deployment of Vision Language Models for real-time video analysis. Using Intel DLStreamer pipelines and OpenVINO-optimized VLMs, the system ingests RTSP video streams, performs AI inference, and provides real-time captions via WebRTC streaming with live performance metrics.

### Use Cases

**Real-time Video Analytics**: Monitor security cameras, industrial equipment, or public spaces with AI-powered scene understanding and automatic captioning.

**Accessibility Enhancement**: Generate live captions for video content, making streams accessible to users with hearing impairments.

**Performance Benchmarking**: Evaluate Vision Language Model performance on Intel® hardware by comparing throughput, latency, and resource utilization across different models and pipeline configurations.

**Intelligent Surveillance**: Deploy custom detection and description pipelines by adjusting prompts (e.g., "Is there a person in the frame?") for security and safety applications.

### Key Features

**Multi-Model Support**: Seamlessly switch between VLMs (InternVL2, Gemma-3, MiniCPM-V, Phi-4) with automatic model discovery and dropdown selection.

**Real-time Streaming**: WebRTC-based video delivery with configurable bitrate and peer-to-peer signaling for low-latency viewing.

**Performance Metrics**: Monitor CPU usage, GPU utilization, power consumption, and inference performance (TTFT, TPOT, throughput) in real-time charts.

**Modular Architecture**: Clearly separated backend (Python FastAPI), frontend (vanilla JavaScript), and pipeline configuration for easy customization and extension.

**Agent Mode**: Optional alert styling for binary classification tasks (red border for "Yes", green for "No") enabling lightweight automation and monitoring workflows.

**Flexible Configuration**: Adjustable prompts, max token limits, and pipeline parameters through an intuitive web dashboard.

### Workflow Overview

**Stream Configuration**: Provide an RTSP URL, select a VLM model, customize the inference prompt, and choose pipeline resources (CPU/GPU).

**AI Processing**: Intel DLStreamer pipeline ingests video at full frame rate, extracts frames at 1fps for AI inference, and applies OpenVINO VLM model for caption generation.

**Real-time Monitoring**: Captions and performance metrics stream to the dashboard via Server-Sent Events (SSE) and WebSocket connections.

**Visualization & Analysis**: View live video, captions, and system metrics (CPU, GPU, memory) on a responsive dashboard with theme support and settings persistence.

## Architecture

### System Components

- **video-ingestion**: Intel DLStreamer Pipeline Server processing RTSP sources with GStreamer pipelines and `gvagenai` for VLM inference
- **mediamtx**: WebRTC/WHIP signaling server for peer-to-peer video delivery
- **coturn**: TURN server for NAT traversal in WebRTC connections
- **app**: Python FastAPI backend serving REST APIs, SSE metadata streams, and WebSocket metrics
- **collector**: Intel VIP-PET system metrics collector (CPU, GPU, memory, power)

### Project Structure

```
app/
├── main.py                          # FastAPI application entry point
├── Dockerfile                       # Container configuration
├── pyproject.toml                   # Python dependencies
│
├── backend/                         # Python API
│   ├── config.py                    # Environment & configuration
│   ├── state.py                     # Application state (active runs)
│   ├── models/                      # Pydantic request/response models
│   ├── services/                    # Business logic (discovery, HTTP, metadata)
│   └── routes/                      # API endpoints (/api/runs, /api/models, /ws/*)
│
└── ui/                              # Frontend
    ├── index.html                   # Main HTML
    ├── css/styles.css               # Styling
    └── js/
        ├── app.js                   # Main entry point
        ├── components/              # Reusable UI components
        ├── services/                # API, metadata, metrics services
        └── utils/                   # Theme, settings, charts utilities
```

### Data Flow

```
RTSP Source → video-ingestion 
            ├─→ 1fps AI branch (GStreamer gvagenai) → /tmp/results.jsonl
            └─→ 30fps preview → mediamtx (WebRTC) → Dashboard
                                                 ↓
                                  Dashboard collects metrics (CPU, GPU, RAM)
```

## Getting Started

### Prerequisites

- Docker and Docker Compose (non-root Docker recommended)
- Host with sufficient CPU/GPU resources for your chosen VLM
- OpenVINO-compatible Vision Language Model in `ov_models/`
- RTSP stream source (live camera or test feed)

### Environment Setup

1. Create `.env` in the repository root:

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

| Variable | Default | Description |
|----------|---------|-------------|
| `HOST_IP` | `localhost` | IP address accessible from remote clients for WebRTC signaling |
| `AGENT_MODE` | `false` | Enable alert styling when caption contains "Yes" or "No" |
| `DASHBOARD_PORT` | `4173` | Dashboard web interface port |
| `METADATA_POLL_SECONDS` | `1` | Metadata stream polling interval |
| `PIPELINE_NAME` | `GenAI_Pipeline_on_CPU` | Default pipeline configuration |
| `PIPELINE_SERVER_URL` | `http://video-ingestion:8080` | DLStreamer Pipeline Server URL |

### Model Setup

2. (Optional) Download and export OpenVINO models:

```bash
chmod +x download_models.sh
./download_models.sh [phi4|minicpm|gemma3|internvl2]
```

For gated models (e.g., MiniCPM-V-2_6), set:

```bash
export HF_TOKEN=<YOUR_HUGGING_FACE_TOKEN>
```

Models are automatically discovered and listed in the dashboard dropdown.

### Starting the Stack

3. Launch all services:

```bash
docker compose up --build
```

Exposed services:
- **Pipeline API**: http://localhost:8040 (REST endpoints for pipeline management)
- **WebRTC Signaling**: ws://localhost:8889 (WHIP/WebRTC peer connections)
- **Dashboard**: http://localhost:4173 (Web interface for monitoring and control)

### Running a Captioning Pipeline

4. Open the dashboard: http://localhost:4173

5. Configure and start a pipeline:
   - **RTSP URL**: Enter a reachable RTSP stream URL
   - **Model**: Select a VLM from the dropdown
   - **Prompt**: Customize the inference prompt
   - **Max Tokens**: Set output length limit (default: 70)
   - **Run Name** (optional): Label for this pipeline run

6. Click **Start** and monitor:
   - Live video feed with captions
   - Real-time metrics (CPU, GPU, memory usage)
   - Inference performance (TTFT, TPOT, throughput)

7. Stop the pipeline by clicking **Stop** on the run card

### Stopping Services

```bash
docker compose down
```

### Metrics Collection

System metrics are collected via the Intel VIP-PET collector container and streamed to the dashboard via WebSocket. Metrics include:

- **CPU**: Per-core and total usage percentage
- **Memory**: RAM utilization percentage
- **GPU**: Engine-specific usage (RCS, Video, etc.), frequency, temperature, power
- **Inference**: Time-To-First-Token (TTFT), Time-Per-Output-Token (TPOT), throughput (tokens/sec)

## API Endpoints

### REST Endpoints

- `GET /api/models` - List available VLM models
- `GET /api/pipelines` - List available pipeline configurations
- `POST /api/runs` - Start a new captioning pipeline
- `GET /api/runs` - List active runs
- `GET /api/runs/{run_id}` - Get run details
- `DELETE /api/runs/{run_id}` - Stop a pipeline
- `GET /api/runs/metadata-stream` - Server-Sent Events stream for captions and metrics

### WebSocket Endpoints

- `ws://localhost:4173/ws/collector` - Metrics collector connection (single connection)
- `ws://localhost:4173/ws/clients` - Metrics broadcast to dashboard clients (multiple connections)

## Additional Resources

- [Intel DLStreamer Documentation](https://docs.openvino.ai/2024/omz_dev_tools_downloader.html)
- [OpenVINO GenAI](https://github.com/openvinotoolkit/openvino.genai)
- [Vision Language Models](https://huggingface.co/models?task=image-to-text)
- [MediaMTX WebRTC Server](https://github.com/bluenviron/mediamtx)

## License

This project is part of the Intel Edge AI Suite and follows the same licensing terms as the parent repository.
