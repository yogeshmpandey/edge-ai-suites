# Copilot Instructions — Smart NVR

## Project Overview

Smart NVR is a GenAI-enabled Network Video Recorder composed of two Python applications sharing one Docker image:
- **Backend** (`src/`): FastAPI app on port 8000 — event routing, rules engine, Redis state, MQTT listener
- **UI** (`ui/`): Gradio app on port 7860 — camera/rule management dashboard

External dependencies: Frigate (VMS), Eclipse Mosquitto (MQTT broker), Redis, and VSS (Video Search & Summarization services).

## Build, Test, and Run

### Tests
```bash
# Run all tests (from project root)
pytest src/tests/

# Run a single test file
pytest src/tests/test_rules_endpoints.py

# Run a single test
pytest src/tests/test_rules_endpoints.py::test_add_rule_success

# With coverage
pytest --cov=src --cov=ui src/tests/
```
`asyncio_mode = "auto"` is set in `pyproject.toml` — no need to manually mark tests with `@pytest.mark.asyncio`.

### Docker
```bash
# Build image
./build.sh

# Start full stack
docker compose -f docker/compose.yaml up
```

The container supports three modes via the `MODE` env var: `backend`, `ui`, or `combined`.

### Install dependencies
```bash
pip install -r requirements.txt
# or with poetry (dev)
poetry install
```

## Architecture

### Event Flow

```
MQTT (Frigate or Scenescape)
  └─► mqtt_listener.py         # paho-mqtt client on daemon thread; bridges to asyncio via
                               # asyncio.run_coroutine_threadsafe()
        └─► rule_engine.py     # load rules from Redis, match label/camera/source/count
              └─► dispatcher.py # dispatch_action("summarize" | "add to search")
                    └─► vms_service.py  # fetch clip from Frigate → upload to VSS
                          └─► redis_store.py  # persist summary_id / search result
```

### Directory Watcher Flow

`directory_watcher.py` watches Frigate recordings (`/media/frigate/recordings` by default) using `watchdog`. When `.mp4` files appear for enabled cameras, it debounces and batches uploads to the VSS search service via `utils/utils.py:upload_videos_to_dataprep`. Camera-watcher enable/disable state is persisted to Redis and restored on startup.

### Key Modules

| Module | Responsibility |
|---|---|
| `src/main.py` | FastAPI app factory; startup hooks (Redis, MQTT, watcher restore) |
| `src/config.py` | All env var reads; no defaults for most external service URLs |
| `src/api/router.py` | All API routes (rules CRUD, cameras, watchers, summary/search) |
| `src/service/redis_store.py` | All Redis I/O (rules, responses, summaries, watcher mapping) |
| `src/service/rule_engine.py` | Rule-matching logic (label, camera, source, count threshold) |
| `src/service/dispatcher.py` | Action dispatch for matched rules |
| `src/service/vms_service.py` | Frigate clip retrieval + VSS upload orchestration |
| `src/service/directory_watcher.py` | `watchdog`-based filesystem watcher with debouncing |
| `src/api/endpoints/frigate_api.py` | `FrigateService` — proxies Frigate REST API |
| `src/api/endpoints/summarization_api.py` | `SummarizationService` — proxies VSS REST API |
| `src/utils/common.py` | `settings` (pydantic-settings) + `logger` for watcher/upload |
| `src/utils/utils.py` | `upload_videos_to_dataprep` with exponential-backoff retry |
| `ui/` | Gradio UI; reads backend via `API_BASE_URL` env var |

## Key Conventions

### Redis Client Pattern
`app.state.redis_client` is the authoritative async client (set at FastAPI startup). Many `redis_store` functions accept an optional `request: Request` parameter and fall back to a module-level `fallback_redis_client` when called outside a request context (e.g., from MQTT callbacks):
```python
redis_client = (
    getattr(request.app.state, "redis_client", None)
    if request
    else fallback_redis_client
)
```
Always pass `request` when inside a FastAPI route handler; omit it when calling from the MQTT or watcher code paths.

### Redis Key Schema
| Key | Type | Content |
|---|---|---|
| `rule:{id}` | string | JSON rule object |
| `rules` | set | all rule IDs |
| `response:{rule_id}` | list | JSON action responses |
| `summary_ids:{rule_id}` | list | VSS summary pipeline IDs |
| `summary_result:{summary_id}` | string | final summary text |
| `search_results:{rule_id}` | list | JSON search result entries |
| `camera_watcher_mapping` | string | JSON `{camera: bool}` map |

### Rule Model
Rules have `id`, `label`, `action` (required) and optional `camera`, `source`, `count`. The `source` field filters events by origin (`"frigate"` or `"scenescape"`). The `count` field sets a minimum vehicle/pedestrian count threshold (SceneScape integration only).

### MQTT Threading Bridge
The MQTT `on_message` callback runs on a paho thread. Async coroutines are dispatched via:
```python
asyncio.run_coroutine_threadsafe(process_event(...), event_loop)
```
where `event_loop` is a dedicated daemon asyncio loop started at module import time. Never call `asyncio.run()` from these callbacks.

### Frigate Recording Directory Layout
```
<root>/<YYYY-MM-DD>/<HH>/<camera_name>/<segment>.mp4
```
The watcher and upload utilities use this layout to extract the camera name from file paths.

### Clip Constraints
- Frigate clips are filtered: minimum event duration is **10 seconds** (`end_time - start_time >= 10`)
- Maximum clip duration is **300 seconds** (enforced by `FrigateService`)
- Clips smaller than 512 KB or 100 bytes are treated as empty/missing

### Testing Patterns
- `src/tests/conftest.py` adds `src/` to `sys.path` and provides a module-scoped `client` fixture (FastAPI `TestClient`)
- External services (Redis, Frigate, VSS) are always mocked with `unittest.mock.patch` and `AsyncMock`
- Patch targets use the import path as seen from the module under test (e.g., `"api.router.redis_store.add_rule"`)

### Environment Variables
All config is read via `os.getenv()` in `src/config.py` (no `.env` file required in prod). Key vars:
- `FRIGATE_BASE_URL`, `VSS_SUMMARY_URL`, `VSS_SEARCH_URL`
- `HOST_IP`, `MQTT_PORT` (default 1884), `REDIS_PORT` (default 6379)
- `MQTT_USER`, `MQTT_PASSWORD`
- `NVR_SCENESCAPE=true` — enables SceneScape MQTT client (requires TLS certs)
- `WATCH_DIRECTORY_CONTAINER_PATH` (default `/media/frigate/recordings`)
- `MODE` — Docker entrypoint mode: `backend`, `ui`, or `combined`

### Copyright Header
All source files must begin with:
```python
# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
```
