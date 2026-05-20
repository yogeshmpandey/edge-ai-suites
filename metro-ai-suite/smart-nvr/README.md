# Smart NVR: GenAI-Enabled Network Video Recorder

The Smart NVR is a next-generation network video recorder that integrates GenAI-powered vision analytics to deliver intelligent, real-time insights from video streams. By processing and analyzing video data directly at the edge, it enables advanced event detection, summarization, and automation while reducing bandwidth and storage requirements. This transformation empowers organizations to extract greater value from their existing video infrastructure and respond rapidly to critical situations.

## Documentation

- **Overview**
  - [Overview](./docs/user-guide/index.md): A high-level introduction.
  - [Overview Architecture](./docs/user-guide/index.md#how-it-works): High-level architecture.

- **Getting Started**
  - [Get Started](./docs/user-guide/get-started.md): Step-by-step guide to get started with the sample application.
  - [System Requirements](./docs/user-guide/get-started/system-requirements.md): Hardware and software requirements for running the sample application.
  - [How to Use the Application](./docs/user-guide/how-to-use-application.md): Explore the application's features and verify its functionality.
  - [Troubleshooting](./docs/user-guide/troubleshooting.md): Support and troubleshooting
  information for the application.

- **Deployment**
  - [Build from Source](./docs/user-guide/get-started/build-from-source.md): Instructions for building from source code.
  - [Deploy with Helm](./docs/user-guide/get-started/deploy-with-helm.md): Instructions for building using helm.

- **Advanced Integrations**
  - [Intel® SceneScape Integration](./docs/user-guide/scenescape-integration.md): Complete guide for integrating with Intel® SceneScape for traffic analytics and vehicle counting.

- **API Reference**
  - [API Reference](./docs/user-guide/api-reference.md): Comprehensive reference for the available REST API endpoints.

- **Release Notes**
  - [Release Notes](./docs/user-guide/release-notes.md): Information on the latest updates, improvements, and bug fixes.

## Single/Dual-Node Demo Deployment

Use `scripts/deploy.sh` to deploy the full SceneScape SmartNVR demo on one/two node: VSS search, MediaMTX RTSP streaming, Frigate recording, SceneScape analytics, and SmartNVR event routing to VSS search.

### Prerequisites

- Docker and Docker Compose v2 available to the current user
- Internet access for GitHub resources, container images, and model assets
- Sufficient disk space for VSS images/models and demo videos
- Hardware that meets the documented SmartNVR and VSS minimum requirements
- If gated models are used, provide `HUGGINGFACE_TOKEN` in the shell environment; the script does not store or print it

### Quick Start

```bash
bash scripts/single_node_deploy.sh
```

The script supplies local demo defaults for VSS credentials and model settings when they are not already exported. To override them, export values before running the script.

### What the Script Does

1. Validates Docker, Docker Compose v2, daemon access, and registry access.
2. Clones `edge-ai-libraries` at tag `v2026.1.0-rc1` and starts VSS in search mode.
3. Downloads the four demo `.ts` videos and verifies them under `resources/videos/`.
4. Configures Frigate to loop-play the videos, expose four RTSP streams, and record clips while detection/snapshots/motion are disabled.
5. Starts MediaMTX and renders Frigate plus SceneScape DL Streamer configs so both consume the same RTSP streams.
6. Prepares SceneScape in the sibling `../metro-vision-ai-app-recipe` app, then starts it with the same flow used manually: `./install.sh smart-intersection` followed by `docker compose up -d`.
7. Starts SmartNVR with SceneScape MQTT and VSS search wiring, seeds demo SceneScape rules, and prints a component status table.

### Re-running the Script

The script is idempotent. It stores deployment state under `.deploy-state/single-node/`, compares generated config hashes, and skips healthy components that are already configured. Use `--force` to regenerate configs and restart services.

### Cleanup

```bash
bash scripts/deploy.sh --cleanup
```

Cleanup stops the SmartNVR, SceneScape, and VSS stacks, restores the previous Frigate config when a backup exists, removes the cloned `edge-ai-libraries` deployment copy, and preserves downloaded videos and SceneScape-generated secrets.

### Troubleshooting

- **Docker permission denied**: Add your user to the `docker` group or run with appropriate privileges.
- **Docker pull fails**: Check internet, registry, and proxy settings.
- **Port conflict on 8554 or 1883**: Stop services using those ports; this demo reserves 8554 for MediaMTX RTSP and 1883 for the SceneScape broker.
- **VSS setup fails**: Check `.deploy-state/single-node/logs/vss-setup-search.log`.
- **Frigate streams missing**: Check the MediaMTX publishers with `docker logs rtsp-publisher` and Frigate ingest with `http://localhost:5000/api/stats`.
- **SceneScape MQTT errors**: Verify SceneScape secrets exist under `../metro-vision-ai-app-recipe/smart-intersection/src/secrets/` and check `docker logs nvr-event-router`.
- **No VSS uploads**: Confirm the seeded `source=scenescape` rules exist in SmartNVR and that SceneScape events are arriving.
