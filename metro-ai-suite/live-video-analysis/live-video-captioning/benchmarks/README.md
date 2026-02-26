# Live Video Captioning Benchmark

## What this benchmark tool does

This benchmark tool runs live video captioning for RTSP streams and records performance data.

For each run it:
- starts a captioning pipeline using `POST /api/runs`
- collects captions + inference metadata from SSE (`/api/runs/metadata-stream`)
- collects system utilization metrics from WebSocket (`/ws/clients`)
- writes per-run outputs (`caption_metrics.csv` and `run_settings.yaml`)

Per-run outputs include:
- `caption_metrics.csv` (one row per generated caption event)
  - run identity: `video_name`, `rest_request_key`, `model_name`, `run_id`, `pipeline_id`
  - caption data: `caption`, `caption_event_ts`, `lag`, `prompt`
    - `lag` = time in seconds between this caption and the previous caption in the same run
  - request params used: `maxNewTokens`, `frameRate`, `chunkSize`, `frameWidth`, `frameHeight`
  - system metrics snapshot at caption time: `cpu_percent`, `ram_percent`, `gpu_percent`, `gpu_temp_c`, `gpu_power_w`, `pkg_power_w`
  - raw inference metrics payload: `sse_metrics_json`
- `run_settings.yaml`
  - full effective request payload sent to `/api/runs`
  - service/execution/collection settings used for that run
  - identifiers (`run_id`, `pipeline_id`, `model_name`, `video_name`, `rest_request_key`)

How many permutations (runs) you can execute:
- Total runs = `(#RTSP URLs) × (#rest_request_* blocks) × (#models)`
- Example: 2 RTSP URLs, 3 `rest_request_*` profiles, 7 models = `2 × 3 × 7 = 42` runs
- Runtime estimate = `Total runs × (run_duration_sec + inter_run_cooldown_sec)`

## Files in this folder

- `run_benchmark.sh` (recommended launcher)
- `run_caption_benchmark.py` (Python runner)
- `requirements-benchmark.txt`
- `configs/fighting_config.yaml` (example config)
- `configs/fighting.txt` (RTSP list)

## Quick start

From the project root:

1. Start stack (if not already running):
   - `docker compose up -d`
2. Run benchmark with shell launcher:
   - `./benchmarks/run_benchmark.sh --config benchmarks/configs/fighting_config.yaml`

If services are already running, skip compose startup:
- `./benchmarks/run_benchmark.sh --config benchmarks/configs/fighting_config.yaml --no-stack`

## Config basics

In `configs/*.yaml`:
- `service.api_base_url`: backend URL (example: `http://10.223.23.199:4173`)
- `inputs.rtsp_list_file`: text file with one RTSP URL per line
- `rest_request_*`: request profiles sent to `/api/runs`
- `execution.run_duration_sec`: benchmark duration per run

## Output structure

Outputs go to `benchmarks/results`.

Layout:
- `<video_name>/`
  - `<rest_request_key>/`
    - `<unique_run_folder>/`
      - `caption_metrics.csv`
      - `run_settings.yaml`

`unique_run_folder` is generated per run, so old CSV/YAML files are never overwritten.

## Common issue

If you see `Pipeline not found`:
- your `rest_request_*.pipelineName` does not match a valid pipeline
- check valid names from:
  - `GET <api_base_url>/api/pipelines`
