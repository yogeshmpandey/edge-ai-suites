# How to Deploy with Helm Chart

This guide shows how to deploy the Live Video Captioning application on Kubernetes with the Helm chart included in this repository.

## Prerequisites

Before you begin, ensure that you have the following:

- A Kubernetes cluster with `kubectl` configured for access.
- Helm installed on your system. See the [Installation Guide](https://helm.sh/docs/intro/install/).
- Dynamic Persistent Volume provisioning available in the cluster, or a `StorageClass` you can set in the chart values.
- A worker node reachable by your browser client. The chart uses this address for dashboard access and WebRTC signaling.
- Sufficient storage for model PVCs. The default chart configuration requests `50Gi` for VLM models and `5Gi` for detection models.
- A writable host path for collector signal files on the target node. By default the chart uses `/tmp/lvc/collector-signals`.
- An RTSP source reachable from the Kubernetes node that runs `dlstreamer-pipeline-server`.
- If you use gated Hugging Face models, a Hugging Face token stored in a Kubernetes secret.

## Prepare the Cluster

### 1. Select the target node

All workloads in this chart are pinned to the target node selected in the chart values.

Set `global.nodeName` to the Kubernetes node name. This uses the built-in `kubernetes.io/hostname` label, so you do not need permission to label nodes.

Example:

```yaml
global:
  nodeName: worker4
```

### 2. Create a namespace

```bash
my_namespace=lvc
kubectl create namespace "$my_namespace"
```

If the namespace already exists, reuse it with the same value.

### 3. Get the IP of the selected node

Use the same node that you selected for this chart. First list the nodes and labels:

```bash
kubectl get nodes --show-labels
```

Then inspect the selected node:

```bash
kubectl get node <node-name> -o wide
```

Set `global.hostIP` to the node address that is reachable by the browser:

- In clusters without worker-node external IPs, use `INTERNAL-IP`.
- Use `EXTERNAL-IP` only if the node actually has one and your browser reaches the application through it.
- Use `INTERNAL-IP` when your browser is on the same LAN or VPN and can reach the node directly.

To print the value directly:

```bash
kubectl get node <node-name> -o jsonpath='{.status.addresses[?(@.type=="ExternalIP")].address}'
```

If no external address is present, use:

```bash
kubectl get node <node-name> -o jsonpath='{.status.addresses[?(@.type=="InternalIP")].address}'
```

Set that value in `global.hostIP`. Do not use a pod IP, a Service `ClusterIP`, or `127.0.0.1` unless the browser runs on the same node.

If the worker node does not have any browser-reachable IP, direct NodePort access will not work. In that case, expose the application through a load balancer, ingress, VPN, SSH tunnel, or another network path that makes the selected node reachable from the browser.

## Configure Required Values

The chart includes a sample override file at `charts/values-override.yaml`. Update it before deploying.

The most important values are:

| Key | Description | Example |
| --- | --- | --- |
| `global.hostIP` | Browser-reachable IP of the selected node. In many on-prem clusters this is the node `INTERNAL-IP`. Retrieve it with `kubectl get node <node-name> -o wide` | `192.168.1.20` |
| `global.nodeName` | Kubernetes node name used to pin workloads to a specific worker node | `worker4` |
| `global.storageClassName` | StorageClass for the chart PVCs. Leave empty to use the cluster default | `local-path` |
| `modelsPvc.size` | PVC size for downloaded or pre-populated VLM models | `50Gi` |
| `detectionModelsPvc.size` | PVC size for object detection models | `5Gi` |
| `modelsDownload.enabled` | Whether the pre-install hook downloads models into the VLM models PVC | `true` |
| `modelsDownload.models` | List of Hugging Face model IDs to export to OpenVINO format | `OpenGVLab/InternVL2-1B` |
| `modelsDownload.hfTokenSecret.name` | Secret name for gated-model downloads | `hf-token` |
| `mediamtx.webrtcIceUsername` | TURN username exposed to the browser | `lvcuser` |
| `mediamtx.webrtcIcePassword` | TURN password exposed to the browser | `lvcpass` |
| `video-caption-service.env.defaultRtspUrl` | Default RTSP URL shown in the dashboard | `rtsp://camera.example/live` |
| `video-caption-service.env.enableDetectionPipeline` | Enables detection filtering in the pipeline | `"true"` or `"false"` |
| `video-caption-service.env.alertMode` | Switches captioning to binary alert-style responses | `"true"` or `"false"` |
| `dlstreamer-pipeline-server.env.detectionDevice` | Device used for object detection inference | `CPU` or `GPU` |
| `collector.collectorSignalsHostPath` | Host path mounted into the collector pod | `/tmp/lvc/collector-signals` |

### Gated Model Downloads

If your selected model requires authentication, create a secret in the target namespace:

```bash
kubectl create secret generic hf-token \
  --from-literal=HF_TOKEN=<your-token> \
  -n "$my_namespace"
```

Then set the following values:

```yaml
modelsDownload:
  hfTokenSecret:
    name: hf-token
    key: HF_TOKEN
```

### Proxy Configuration

If your cluster runs behind a proxy, set the proxy fields under `global`:

```yaml
global:
  httpProxy: "http://<your-proxy-host>:<port>"
  httpsProxy: "http://<your-proxy-host>:<port>"
  noProxy: "<your-rtsp-camera-host-or-ip>"
```

Important: the host portion of every RTSP URL must be included in `noProxy` when the deployment runs behind a proxy.

For example:

- If your stream URL is `rtsp://camera.example.com:8554/live`, add `camera.example.com` to `noProxy`.
- If your stream URL is `rtsp://192.168.1.50:554/stream1`, add `192.168.1.50` to `noProxy`.

If the RTSP host is not listed in `noProxy`, the application may try to reach the stream through the proxy and fail to connect.

## Build Chart Dependencies

Run the following command from the chart directory:

```bash
helm dependency update
```

This refreshes the packaged dependencies from `charts/subcharts/` and updates `Chart.lock`.

## Install the Chart

From `charst`, install the application with the override file:

```bash
helm install lvc . \
  -f values-override.yaml \
  -n "$my_namespace" \
  --timeout 60m
```

You can also install from the repository root:

```bash
helm install lvc ./charts \
  -f ./charts/values-override.yaml \
  -n "$my_namespace" \
  --timeout 60m
```

> **Note:** The `--timeout 60m` flag is required because the pre-install hook downloads and converts
> OpenVINO models, which can take 30–60+ minutes depending on network speed and model size.
> Helm's default timeout is 5 minutes and will cause the install to fail prematurely.

## Verify the Deployment

Check the hook job, pods, services, and PVCs:

```bash
kubectl get jobs,pods,svc,pvc -n "$my_namespace"
```

The model downloader runs as a Helm hook before the main workloads start. If the initial deployment takes time, inspect the job logs:

```bash
kubectl logs -n "$my_namespace" -l app.kubernetes.io/component=model-downloader
```

Before accessing the application, confirm the following:

- The model download job has completed successfully or was intentionally disabled.
- All pods are in the `Running` state.
- All containers report `Ready`.
- The PVCs are bound.

The first deployment can take several minutes because the chart may download and export VLM models before starting the application pods.

## Access the Application

By default the chart exposes these NodePort services:

- Dashboard UI: `http://<global.hostIP>:4173`
- DL Streamer Pipeline API: `http://<global.hostIP>:8040`
- Live Metrics endpoint: `http://<global.hostIP>:9090`
- MediaMTX WHIP / WebRTC signaling: `http://<global.hostIP>:8889`

If you changed the service ports in your override values, use those instead.

To start captioning after deployment:

1. Open the dashboard URL in your browser.
2. Enter an RTSP stream URL, unless you preconfigured `defaultRtspUrl`.
3. Select the model you downloaded into the models PVC.
4. Adjust the prompt and generation parameters if needed.
5. Start the stream.

## Upgrade the Release

If you modify the chart or subcharts, refresh dependencies first:

```bash
helm dependency update
```

Then upgrade the release:

```bash
helm upgrade lvc . \
  -f values-override.yaml \
  -n "$my_namespace" \
  --timeout 60m
```

## Uninstall the Release

```bash
helm uninstall lvc -n "$my_namespace"
```

## Troubleshooting

- If pods remain `Pending`, check that `global.nodeName` matches the correct node name and that the requested `StorageClass` can provision the PVCs.
- If the install fails before pods appear, inspect the model download hook logs and confirm that the selected model ID and Hugging Face credentials are valid.
- If the dashboard opens but video does not start, confirm that `global.hostIP` is reachable from the browser. If your worker nodes do not have external IPs, this usually means using the node `INTERNAL-IP` over a reachable LAN or VPN. Also confirm that the RTSP source is reachable from the Kubernetes node.
- If WebRTC negotiation fails, verify that the NodePort services for MediaMTX and coturn are allowed by your network policy or firewall.
- If detection is enabled but the pipeline cannot start, ensure the detection models PVC contains the required OpenVINO detection model artifacts.
- If the collector does not report metrics, confirm that the host path in `collector.collectorSignalsHostPath` exists on the selected node and that the pod is scheduled there.

## Related Links

- [Get Started](./get-started.md)
- [System Requirements](./get-started/system-requirements.md)
- [How it Works](./how-it-works.md)
- [Object Detection Pipeline](./object-detection-pipeline.md)
- [Build from Source](./get-started/build-from-source.md)