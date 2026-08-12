# Multiple SceneScape Deployment

Smart NVR with SceneScape running on separate machines: Smart Intersection (SI) on
System 1, NVR stack on System 2. For single-node deployment, see
[Integrate SceneScape with Smart NVR](./scenescape-integration.md).

## Overview

Smart NVR maintains a persistent, independent MQTT connection to each SI node.
Events are tagged with the broker `id` to route them to the correct Frigate camera.

- **One file configures everything.** `intersections.yaml` is the single source of
  configuration: each entry holds the intersection name, its IP and its 4 cameras.
  The same file drives both the MQTT broker connections and the Frigate RTSP inputs,
  so `setup.sh` never prompts for IP addresses.
- **Camera naming.** Frigate cameras must be named `{id}-camera{n}`
  (e.g. `si1-camera1`). The intersection `id` must match this prefix exactly.
- **Config persists.** On startup, the broker manager reads `intersections.yaml`,
  seeds Redis, and starts one connection per enabled intersection. The `/brokers/`
  API persists changes back to this file.

## Prerequisites

- VSS must be running and reachable from System 2.
- See [system requirements](./get-started/system-requirements.md) for hardware prerequisites.

## Configuration

### intersections.yaml

Edit `resources/broker-config/intersections.yaml` before starting. This is the only
place RTSP and MQTT endpoints are configured — the setup script reads it instead of
prompting. Brokers can still be managed at runtime via the API, and the file is
updated automatically when they change.

```yaml
# resources/broker-config/intersections.yaml
intersections:
  - id: si1                          # must match the Frigate camera prefix: si1-camera*
    name: Main Street and 1st Ave    # human-readable intersection name
    ip: 10.0.0.11                    # MQTT broker for this intersection
    cameras:
      - name: si1-camera1
        url: rtsp://10.0.0.21:8554/camera1   # each camera is a self-contained RTSP URL
      - name: si1-camera2
        url: rtsp://10.0.0.22:8554/camera1
      - name: si1-camera3
        url: rtsp://10.0.0.23:8554/camera1
      - name: si1-camera4
        url: rtsp://10.0.0.24:8554/camera1

  - id: si2
    name: Broadway and 5th
    ip: 10.0.0.12
    cameras:
      - name: si2-camera1
        url: rtsp://10.0.0.31:8554/camera1
      - name: si2-camera2
        url: rtsp://10.0.0.32:8554/camera1
      - name: si2-camera3
        url: rtsp://10.0.0.33:8554/camera1
      - name: si2-camera4
        url: rtsp://10.0.0.34:8554/camera1
```

> TLS is enabled by default. Broker connections do not use username or password authentication.

### Intersection fields

| Field | Required | Default | Description |
|-------|----------|---------|-------------|
| `ip` | ✅ | — | Intersection IP. Used as the MQTT broker host. |
| `id` | — | camera prefix | Unique identifier, e.g. `si1` → cameras `si1-camera*`. Derived from the camera names when omitted. |
| `name` | — | `id` | Human-readable intersection name. |
| `cameras` | — | 4 generated | The cameras of the intersection. Defaults to `<id>-camera1` .. `<id>-camera4`. |
| `mqtt_port` | — | `1883` | MQTT broker port. |
| `topic` | — | `scenescape/data/camera/#` | MQTT topic to subscribe to. |
| `use_tls` | — | `true` | Enable TLS. Set to `false` for plain MQTT brokers. |
| `throttle_interval` | — | `2.0` | Minimum seconds between processed events. |
| `enabled` | — | `true` | Set to `false` to skip the intersection (no MQTT connection, no Frigate cameras). |

### Camera fields

| Field | Required | Default | Description |
|-------|----------|---------|-------------|
| `name` | ✅ | — | Frigate camera name. Must start with the intersection id, e.g. `si1-camera1`. |
| `url` | — | `rtsp://<intersection ip>:<RTSP_STREAM_PORT>/<camera-name suffix>` | Full RTSP source URL for this camera, e.g. `rtsp://10.0.0.21:8554/camera1`. Each camera is self-contained — cameras commonly live on different hosts, so there is no shared ip/port to inherit beyond this same-host convenience default. |

### Environment variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `NVR_SCENESCAPE` | ✅ | — | Must be `true` to enable SceneScape mode. |
| `VSS_IP` | ✅ | — | VSS service IP. The single nginx proxy serves both summary and search. |
| `VSS_PORT` | — | `12345` | VSS service port. |
| `MQTT_USER` | — | auto-generated | Local Mosquitto username (Frigate ↔ NVR). |
| `MQTT_PASSWORD` | — | auto-generated | Local Mosquitto password. |
| `SI_RTSP_HOST` | — | host IP | RTSP source used by the SI node itself (System 1 only). |
| `INTERSECTIONS_AUTO_CONFIRM` | — | — | Set to `true` to skip the "use these intersections?" prompt. |
| `RTSP_STREAM_PORT` | — | `8554` | RTSP port for all SI streams. |
| `SCENESCAPE_MQTT_BROKER` | — | — | Legacy: seeds si1 MQTT broker into Redis when `intersections.yaml` is empty. |
| `INTERSECTIONS_CONFIG_PATH` | — | `resources/broker-config/intersections.yaml` | Path to the intersections config file. |
| `MAX_CONCURRENT_EVENTS` | — | `50` | Maximum simultaneous in-flight event tasks. |
| `BROKER_RECONNECT_DELAY` | — | `5.0` | Seconds before reconnecting after a broker disconnect. |

## Deployment

### System 1 — SI node(s)

```bash
export NVR_SCENESCAPE=true
# export SI_RTSP_HOST=<external_rtsp_ip>  # optional: use an external RTSP source
# export RTSP_STREAM_PORT=<port>              # optional, default 8554
source setup.sh start-si
```

Downloads demo videos and starts a local MediaMTX RTSP streamer by default.
Setting `SI_RTSP_HOST` to a remote IP skips the local streamer.

On exit, the script prints System 1's IP and a ready-to-paste `intersections.yaml`
entry — copy it to System 2.

### System 2 — NVR node

1. Add one entry per intersection to `resources/broker-config/intersections.yaml`
   (see [intersections.yaml](#intersectionsyaml)).
2. Start the stack:

```bash
export NVR_SCENESCAPE=true
export VSS_IP=<ip>
export VSS_PORT=<port>              # optional, default 12345

source setup.sh start-nvr
```

`start-nvr` reads the file and asks for confirmation:

```
Info: Found 3 preconfigured intersection(s) in ./resources/broker-config/intersections.yaml:
  1. si1 - Main Street and 1st Ave @ 10.0.0.11 (4 cameras)
  2. si2 - Broadway and 5th @ 10.0.0.12 (4 cameras)
  3. si3 - Park Ave and 9th @ 10.0.0.13 (4 cameras)
Would you like to use them? (Y/N) [Y]:
```

Answer `Y` to continue, or `N` to stop, update `intersections.yaml` and re-run the
command. Export `INTERSECTIONS_AUTO_CONFIRM=true` to skip the prompt in automated
runs.

> **Note:** `start-nvr` fails fast when `intersections.yaml` has no entries — the
> file must be populated before the NVR node can record or subscribe. In single-node
> mode (`setup.sh start`), a default `si1` entry pointing at the local host is
> created automatically.

## Stop

```bash
source setup.sh stop-nvr   # System 2
source setup.sh stop-si    # System 1
```

If a local RTSP streamer is running on System 1, `stop-si` prompts:

```
Local RTSP streamer is running. Stop it too? [y/N]
```

Respond `y` to stop it, or `n` to leave it running (`source setup.sh stop-streamer`
stops it independently).

## RTSP Streamer

To manage the MediaMTX RTSP streamer on System 1 independently of SI services.
`start-streamer` downloads demo videos if not already present, then starts the streamer.

```bash
source setup.sh start-streamer
source setup.sh stop-streamer
```

## Managing brokers at runtime

The `/brokers/` API modifies live broker connections without restarting the stack.
Changes persist to `intersections.yaml` automatically; existing camera definitions
are preserved, and brokers added through the API get four default cameras pointing
at the broker host.

```bash
BASE=http://localhost:8000

# List brokers
curl $BASE/brokers/

# Add a broker (starts MQTT connection immediately)
curl -X POST $BASE/brokers/ \
  -H "Content-Type: application/json" \
  -d '{
    "id": "si3",
    "name": "Smart Intersection 3",
    "host": "<si3_ip>",
    "topic": "scenescape/data/camera/#"
  }'

# Update a broker (restarts its MQTT connection)
curl -X PUT $BASE/brokers/si3 \
  -H "Content-Type: application/json" \
  -d '{
    "id": "si3",
    "name": "SI 3 updated",
    "host": "<si3_ip_updated>",
    "topic": "scenescape/data/camera/#",
    "enabled": true
  }'

# Remove a broker (stops its MQTT connection)
curl -X DELETE $BASE/brokers/si3
```

> Adding a broker via the API updates MQTT routing only. To record video from a new
> SI node, re-run `setup.sh start-nvr` to regenerate Frigate camera blocks from
> `intersections.yaml`.

## Frigate camera configuration

`setup.sh` generates `resources/frigate-config/config.yml` at startup from
`intersections.yaml` — one camera block per camera, named after the camera `name`:

```yaml
si1-camera1:
  ffmpeg:
    inputs:
      - path: rtsp://10.0.0.21:8554/camera1   # the camera's "url" field, verbatim
```

Each camera's `path` is exactly its `url` field. When a camera omits `url`, one is
derived as `rtsp://<intersection ip>:<RTSP_STREAM_PORT>/<camera-name suffix>` as a
same-host convenience default. Intersections marked `enabled: false` are skipped.

## Verify integration

```bash
# Confirm all broker tasks started
docker logs nvr-event-router | grep "subscribed to"
# Expected:
#   [si1] subscribed to scenescape/data/camera/# at <si1_ip>:1883
#   [si2] subscribed to scenescape/data/camera/# at <si2_ip>:1883

# Monitor live events
docker logs nvr-event-router -f | grep "Scenescape event"

# Check reconnection attempts
docker logs nvr-event-router | grep "reconnecting"
```

## Troubleshooting

**Broker connects but no events appear**

- Verify the broker `id` matches the Frigate camera prefix (e.g. `id: si2` → `si2-camera1..4`).
- Confirm SI is publishing to `scenescape/data/camera/#`.

**`[siN] connection error: [Errno 111] Connect call failed`**

MQTT port unreachable. The broker manager retries automatically. Check connectivity:

```bash
nc -zv <siN_host> 1883
```

**Frigate cameras show no recordings**

RTSP IPs are written into `config.yml` at startup from `intersections.yaml`. If they
changed, update the file and re-run `setup.sh start-nvr` to regenerate the config.

**`Error: No intersections configured`**

`intersections.yaml` is empty or missing. Add at least one intersection (name, ip and
its cameras) and re-run the command.

**UI shows no SceneScape source**

```bash
docker exec nvr-event-router-ui env | grep NVR_SCENESCAPE
```

Confirm `NVR_SCENESCAPE=true` is exported in the shell running `setup.sh start-nvr`.

For general issues, see the [Troubleshooting Guide](./troubleshooting.md).
