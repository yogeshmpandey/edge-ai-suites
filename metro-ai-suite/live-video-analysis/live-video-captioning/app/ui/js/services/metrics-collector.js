/**
 * Metrics collector service for the bundled metrics-manager microservice.
 *
 * Consumes the Server-Sent Events stream exposed by metrics-manager
 * (GET /metrics/stream on port 9090) and renders CPU, RAM, GPU and NPU usage.
 *
 * The stream emits events shaped as:
 *   { "timestamp": <ms>, "metrics": [ { "name", "labels", "value", "timestamp" }, ... ] }
 * where metric names are flattened (measurement + "_" + field), e.g.
 * "cpu_usage_user", "mem_used_percent", "gpu_engine_usage_usage", "npu_utilization".
 */
const MetricsCollectorService = (function () {
    let metricsSource = null;
    let reconnectTimeout = null;
    let reconnectAttempts = 0;
    const maxReconnectAttempts = 10;
    const reconnectDelay = 3000;

    // Metrics service configuration - uses runtime config from backend
    function getMetricsServiceUrl() {
        // Check for explicit metrics service URL configuration
        if (window.METRICS_SERVICE_URL) {
            return window.METRICS_SERVICE_URL;
        }

        const cfg = window.RUNTIME_CONFIG || {};
        // metrics-manager serves the SSE stream over plain HTTP(S), not WS.
        const protocol = window.location.protocol === 'https:' ? 'https:' : 'http:';
        const host = window.location.hostname;
        const port = cfg.metricsServicePort || window.METRICS_SERVICE_PORT || '9090';
        return `${protocol}//${host}:${port}/metrics/stream`;
    }

    function formatEngineName(name) {
        // Format engine names for display (e.g., "rcs0" -> "RCS0", "video" -> "Video")
        if (!name) return 'Unknown';
        return name.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
    }

    function processCollectorMetrics(metrics, elements) {
        const { cpuVal, ramVal, gpuVal, gpuDetail, gpuEngines, gpuFreq, gpuPower, gpuTemp, gpuError, npuVal, npuStat } = elements;

        // Per-batch accumulators
        const gpuEngineData = new Map();
        let gpuPowerValue = null;
        let pkgPowerValue = null;
        let gpuFreqValue = null;
        let gpuTempValue = null;
        let npuUtilization = null;

        metrics.forEach(metric => {
            const { name, value } = metric;
            const labels = metric.labels || {};

            switch (name) {
                case 'cpu_usage_user':
                    // Prefer the aggregate "cpu-total" series when present.
                    if (labels.cpu === undefined || labels.cpu === 'cpu-total') {
                        ChartManager.pushStatSample('cpu', value);
                        if (cpuVal) cpuVal.textContent = `${value.toFixed(1)}%`;
                    }
                    break;

                case 'mem_used_percent':
                    ChartManager.pushStatSample('ram', value);
                    if (ramVal) ramVal.textContent = `${value.toFixed(1)}%`;
                    break;

                case 'gpu_engine_usage_usage':
                    if (labels.engine) {
                        gpuEngineData.set(labels.engine.toUpperCase(), value);
                    }
                    break;

                case 'gpu_frequency':
                    if (labels.type === undefined || labels.type === 'cur_freq') {
                        gpuFreqValue = value;
                    }
                    break;

                case 'gpu_power':
                    if (labels.type === 'gpu_cur_power') {
                        gpuPowerValue = value;
                    } else if (labels.type === 'pkg_cur_power') {
                        pkgPowerValue = value;
                    } else if (gpuPowerValue === null) {
                        gpuPowerValue = value;
                    }
                    break;

                case 'temp_temp':
                    if (typeof labels.sensor === 'string' && labels.sensor.includes('package')) {
                        gpuTempValue = value;
                    }
                    break;

                case 'npu_utilization':
                    npuUtilization = value;
                    break;
            }
        });

        // GPU frequency / temperature detail lines
        if (gpuFreq && gpuFreqValue !== null) {
            gpuFreq.textContent = `Freq: ${gpuFreqValue} MHz`;
            gpuFreq.style.display = 'block';
        }
        if (gpuTemp && gpuTempValue !== null) {
            gpuTemp.textContent = `Temp: ${gpuTempValue}°C`;
            gpuTemp.style.display = 'block';
        }

        // GPU power display
        if (gpuPower && gpuPowerValue !== null) {
            let powerText = `Power: ${gpuPowerValue.toFixed(1)}W`;
            if (pkgPowerValue !== null) {
                powerText += ` (Pkg: ${pkgPowerValue.toFixed(1)}W)`;
            }
            gpuPower.textContent = powerText;
            gpuPower.style.display = 'block';
        }

        // GPU engines breakdown + overall usage (max across engines)
        const engineNames = Array.from(gpuEngineData.keys());
        if (engineNames.length > 0) {
            if (gpuEngines) {
                const engineList = engineNames
                    .map(n => `${formatEngineName(n)}: ${gpuEngineData.get(n).toFixed(1)}%`)
                    .join(' | ');
                gpuEngines.textContent = engineList;
                gpuEngines.style.display = 'block';
            }

            const maxGpuUsage = Math.max(...Array.from(gpuEngineData.values()));
            ChartManager.pushStatSample('gpu', maxGpuUsage);
            if (gpuVal) gpuVal.textContent = `${maxGpuUsage.toFixed(1)}%`;

            // Mark GPU as available
            if (gpuDetail) gpuDetail.style.display = 'block';
            if (gpuError) gpuError.style.display = 'none';
        }

        // NPU usage (only when NPU metrics arrive)
        if (npuUtilization !== null) {
            ChartManager.pushStatSample('npu', npuUtilization);

            if (npuVal) {
                npuVal.textContent = `${npuUtilization.toFixed(1)}%`;
            }
            // Reveal the NPU stat only once data is present
            if (npuStat) npuStat.style.display = '';
        }
    }

    function setConnectionStatus(connected) {
        const collectorStatus = document.getElementById('collectorStatus');
        const collectorStatusDot = document.getElementById('collectorStatusDot');
        if (collectorStatus) {
            collectorStatus.textContent = connected ? 'Connected' : 'Disconnected';
            collectorStatus.className = connected ? 'status-connected' : 'status-disconnected';
        }
        if (collectorStatusDot) {
            collectorStatusDot.classList.toggle('active', connected);
        }
    }

    function init(elements) {
        // Initialize consolidated chart
        ChartManager.createConsolidatedChart('statsChart', [
            { label: 'CPU %', color: '#1ad0ff' },
            { label: 'RAM %', color: '#8ca0c2' },
            { label: 'GPU %', color: '#ffb347' },
            { label: 'NPU %', color: '#b388ff' },
        ]);

        const streamUrl = getMetricsServiceUrl();

        function connectMetricsStream() {
            if (metricsSource && metricsSource.readyState !== EventSource.CLOSED) {
                console.log('Metrics SSE already connected or connecting');
                return;
            }

            console.log('Connecting to metrics-manager SSE stream:', streamUrl);
            metricsSource = new EventSource(streamUrl);

            metricsSource.onopen = () => {
                console.log('Metrics SSE connected');
                reconnectAttempts = 0;
                setConnectionStatus(true);
            };

            metricsSource.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (!data.metrics || !Array.isArray(data.metrics)) {
                        return;
                    }
                    processCollectorMetrics(data.metrics, elements);
                } catch (err) {
                    console.error('Error parsing metrics message:', err);
                }
            };

            metricsSource.onerror = () => {
                // EventSource auto-reconnects while the connection is open; only
                // intervene with manual backoff once it has fully closed.
                setConnectionStatus(false);

                if (metricsSource.readyState === EventSource.CLOSED) {
                    metricsSource.close();
                    metricsSource = null;

                    if (reconnectAttempts < maxReconnectAttempts) {
                        reconnectAttempts++;
                        console.log(`Attempting to reconnect (${reconnectAttempts}/${maxReconnectAttempts})...`);
                        reconnectTimeout = setTimeout(connectMetricsStream, reconnectDelay);
                    } else {
                        console.error('Max reconnect attempts reached for metrics SSE');
                    }
                }
            };
        }

        // Start connection
        connectMetricsStream();

        // Cleanup on page unload
        window.addEventListener('beforeunload', () => {
            if (reconnectTimeout) {
                clearTimeout(reconnectTimeout);
            }
            if (metricsSource) {
                metricsSource.close();
            }
        });
    }

    return {
        init
    };
})();
