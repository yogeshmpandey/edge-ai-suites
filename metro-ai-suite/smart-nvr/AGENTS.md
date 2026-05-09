# SmartNVR Agent Instructions

## Build, Test, and Run

```bash
pytest src/tests/
pytest src/tests/test_rules_endpoints.py
pytest src/tests/test_rules_endpoints.py::test_add_rule_success
pytest --cov=src --cov=ui src/tests/

./build.sh
docker compose -f docker/compose.yaml up -d
```

The runtime image supports `MODE=backend`, `MODE=ui`, and `MODE=combined`.

## Architecture

SmartNVR is a FastAPI backend (`src/`) plus Gradio UI (`ui/`) that integrates with Frigate, Redis, MQTT, VSS, and optionally Intel SceneScape.

SceneScape event flow:

```text
SceneScape MQTT
  -> service/mqtt_listener.py
  -> service/rule_engine.py
  -> service/dispatcher.py
  -> service/vms_service.py
  -> VSS search API
```

For single-node demo deployment, Frigate is the video restreamer and recorder. SceneScape consumes Frigate RTSP streams, publishes metadata to its MQTT broker, and SmartNVR subscribes with `NVR_SCENESCAPE=true`.

## Codebase Conventions

- Source imports assume `src/` is on `PYTHONPATH`; tests add it in `src/tests/conftest.py`.
- Redis access usually accepts an optional FastAPI `Request`; pass `request` in route handlers and omit it in MQTT/watcher paths.
- Rules support `source` and `count`; SceneScape rules should use `source=scenescape`, camera IDs matching SceneScape (`camera1`-`camera4`), and action `"add to search"` for VSS indexing.
- MQTT callbacks run on paho threads; schedule async work with `asyncio.run_coroutine_threadsafe`, not `asyncio.run`.
- Frigate recordings are expected under `/media/frigate/recordings` with layout `<date>/<hour>/<camera>/<segment>.mp4`.
- Do not commit or log tokens. In particular, `HUGGINGFACE_TOKEN` must only be read from the environment.

## Single-Node Deployment Notes

- `scripts/single_node_deploy.sh` owns generated demo state under `.deploy-state/single-node/`.
- The script should be idempotent: check rendered config hashes and container health before restarting services.
- Recommended port policy:
  - Frigate owns host RTSP port `8554`.
  - SceneScape broker owns host MQTT port `1883`.
  - SmartNVR MQTT broker remains on host `1884` for Frigate events.
- The script uses the sibling `../metro-vision-ai-app-recipe` path for SceneScape/Smart Intersection assets.
- Do not overwrite user edits outside generated deployment artifacts unless `--force` is provided.
