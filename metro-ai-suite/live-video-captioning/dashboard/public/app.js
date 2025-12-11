(function () {
    const cfg = window.RUNTIME_CONFIG || {};
    const els = {
        frame: document.getElementById('webrtcFrame'),
        statusDot: document.getElementById('videoStatus'),
        statusText: document.getElementById('statusText'),
        captionText: document.getElementById('captionText'),
        watcherStatus: document.getElementById('watcherStatus'),
        ttftChip: document.getElementById('ttftChip'),
        tpotChip: document.getElementById('tpotChip'),
        throughputChip: document.getElementById('throughputChip'),
        timestampEl: document.getElementById('timestamp'),
        hintEl: document.getElementById('hint'),
        form: document.getElementById('pipelineForm'),
        rtspInput: document.getElementById('rtspInput'),
        startBtn: document.getElementById('startBtn'),
        stopBtn: document.getElementById('stopBtn'),
        pipelineInfo: document.getElementById('pipelineInfo'),
    };

    const state = { pipelineId: null, peerId: cfg.peerId || 'stream' };
    const statsCharts = {};

    function createStatChart(elId, label, color) {
        const ctx = document.getElementById(elId)?.getContext('2d');
        if (!ctx) return null;
        const gradient = ctx.createLinearGradient(0, 0, 0, 140);
        gradient.addColorStop(0, `${color}55`);
        gradient.addColorStop(1, `${color}0f`);
        return new Chart(ctx, {
            type: 'line',
            data: { labels: [], datasets: [{ label, data: [], borderColor: color, backgroundColor: gradient, tension: 0.35, fill: true, pointRadius: 0, borderWidth: 2 }] },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: { display: false },
                    y: {
                        suggestedMin: 0,
                        suggestedMax: 100,
                        grid: { color: 'rgba(255,255,255,0.05)' },
                        ticks: { color: 'rgba(255,255,255,0.55)' },
                    },
                },
                plugins: { legend: { display: false } },
            },
        });
    }

    function pushStatSample(key, value) {
        const chart = statsCharts[key];
        if (!chart) return;
        const maxPoints = 60;
        const labels = chart.data.labels;
        labels.push(new Date().toLocaleTimeString());
        if (labels.length > maxPoints) labels.shift();
        const ds = chart.data.datasets[0];
        ds.data.push(value);
        if (ds.data.length > maxPoints) ds.data.shift();
        chart.update('none');
    }

    function resolveSignalingBase(url) {
        if (!url) return '';
        let base = url.replace(/\/$/, '');
        try {
            const parsed = new URL(base, window.location.origin);
            const localHosts = ['localhost', '127.0.0.1', '0.0.0.0'];
            if (localHosts.includes(parsed.hostname)) parsed.hostname = window.location.hostname;
            base = `${parsed.protocol}//${parsed.hostname}${parsed.port ? ':' + parsed.port : ''}`;
        } catch (_err) {
            base = base.replace('localhost', window.location.hostname);
        }
        return base;
    }

    function setVideoStatus(active, text) {
        els.statusText.textContent = text;
        els.statusDot.classList.toggle('active', !!active);
    }

    function renderCaption(raw) {
        try {
            const payload = JSON.parse(raw);
            if (payload.result) return payload.result;
        } catch (_err) {}
        return raw;
    }

    function renderMetrics(raw) {
        try {
            const payload = JSON.parse(raw);
            const metrics = payload.metrics || {};
            const throughput = metrics.throughput_mean;
            els.ttftChip.textContent = metrics.ttft_mean ? `${metrics.ttft_mean.toFixed(0)} ms` : '—';
            els.tpotChip.textContent = metrics.tpot_mean ? `${metrics.tpot_mean.toFixed(2)} ms` : '—';
            els.throughputChip.textContent = throughput ? `${throughput.toFixed(2)} tok/s` : '—';
            if (payload.timestamp_seconds !== undefined) {
                els.timestampEl.textContent = `Updated ${payload.timestamp_seconds.toFixed(2)}s into stream`;
            } else if (payload.timestamp) {
                const ts = new Date(payload.timestamp);
                els.timestampEl.textContent = `Updated at ${ts.toLocaleTimeString()}`;
            } else {
                els.timestampEl.textContent = '—';
            }
        } catch (_err) {
            els.ttftChip.textContent = '—';
            els.tpotChip.textContent = '—';
            els.throughputChip.textContent = '—';
            els.timestampEl.textContent = '—';
        }
    }

    function updateWatcher(text) {
        if (els.watcherStatus) els.watcherStatus.textContent = text;
    }

    function updatePipelineInfo(text) {
        els.pipelineInfo.textContent = text;
    }

    function initFrame() {
        const base = resolveSignalingBase(cfg.signalingUrl);
        if (!base || !state.peerId) return;
        els.frame.src = `${base}/${state.peerId}`;
        setVideoStatus(false, `Connecting to ${state.peerId}...`);
    }

    function initSSE() {
        const source = new EventSource('/metadata-stream');
        source.onmessage = (event) => {
            if (!event.data) {
                updateWatcher('No data yet');
                els.hintEl.textContent = 'Waiting for metadata...';
                return;
            }
            setVideoStatus(true, 'Streaming...');
            updateWatcher('Receiving data...');
            els.captionText.textContent = renderCaption(event.data);
            renderMetrics(event.data);
            els.hintEl.textContent = '';
        };
        source.onerror = () => {
            updateWatcher('Reconnecting...');
            setVideoStatus(false, 'Retrying WebRTC...');
            els.hintEl.textContent = 'Stream not found, retrying...';
        };
    }

    function initSystemStats() {
        const cpuVal = document.getElementById('cpuVal');
        const ramVal = document.getElementById('ramVal');
        const ramDetail = document.getElementById('ramDetail');
        const gpuVal = document.getElementById('gpuVal');
        statsCharts.cpu = createStatChart('cpuChart', 'CPU %', '#1ad0ff');
        statsCharts.ram = createStatChart('ramChart', 'RAM %', '#8ca0c2');
        statsCharts.gpu = createStatChart('gpuChart', 'GPU %', '#ffb347');
        const statsSource = new EventSource('/system-stats');
        statsSource.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                const cpu = data.cpu_percent ?? 0;
                const memPct = data.mem_percent ?? 0;
                const memUsed = data.mem_used_gb ?? 0;
                const memTotal = data.mem_total_gb ?? 0;
                const gpuPct = data.gpu_percent ?? 0;
                cpuVal.textContent = `${cpu.toFixed(1)}%`;
                ramVal.textContent = `${memPct.toFixed(1)}%`;
                ramDetail.textContent = `${memUsed.toFixed(1)} / ${memTotal.toFixed(1)} GB`;
                gpuVal.textContent = data.gpu_percent == null ? 'n/a' : `${gpuPct.toFixed(1)}%`;
                pushStatSample('cpu', cpu);
                pushStatSample('ram', memPct);
                if (data.gpu_percent != null) pushStatSample('gpu', gpuPct);
            } catch (_err) {
                cpuVal.textContent = '—';
                ramVal.textContent = '—';
                ramDetail.textContent = '—';
                gpuVal.textContent = '—';
            }
        };
    }

    async function startPipeline(evt) {
        evt.preventDefault();
        const rtspUrl = els.rtspInput.value.trim();
        if (!rtspUrl) return;
        els.startBtn.disabled = true;
        updatePipelineInfo('Starting pipeline...');
        const apiBase = `${window.location.protocol}//${window.location.hostname}:8040`;
        const payload = {
            source: { uri: rtspUrl, type: 'uri' },
            destination: {
                metadata: { type: 'file', path: '/tmp/results.jsonl', format: 'json-lines' },
                frame: { type: 'webrtc', 'peer-id': state.peerId, bitrate: 5000 }
            }
        };
        try {
            const resp = await fetch(`${apiBase}/pipelines/user_defined_pipelines/genai_pipeline`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });
            const text = await resp.text();
            if (!resp.ok) throw new Error(text || resp.statusText);
            state.pipelineId = text.replace(/\"/g, '').trim();
            updatePipelineInfo(`Running Pipeline ID: ${state.pipelineId}`);
            els.stopBtn.disabled = false;
            els.startBtn.disabled = true;
        } catch (err) {
            updatePipelineInfo(`Start failed: ${err.message}`);
            state.pipelineId = null;
            els.stopBtn.disabled = true;
        } finally {
            if (!state.pipelineId) {
                els.startBtn.disabled = false;
            }
        }
    }

    async function stopPipeline() {
        if (!state.pipelineId) {
            updatePipelineInfo('No pipeline to stop');
            return;
        }
        els.stopBtn.disabled = true;
        updatePipelineInfo(`Stopping: ${state.pipelineId}`);
        const apiBase = `${window.location.protocol}//${window.location.hostname}:8040`;
        try {
            const resp = await fetch(`${apiBase}/pipelines/${state.pipelineId}`, { method: 'DELETE' });
            if (!resp.ok) {
                const text = await resp.text();
                throw new Error(text || resp.statusText);
            }
            updatePipelineInfo('Pipeline stopped');
            state.pipelineId = null;
        } catch (err) {
            updatePipelineInfo(`Stop failed: ${err.message}`);
        } finally {
            els.stopBtn.disabled = !state.pipelineId;
            if (!state.pipelineId) {
                els.startBtn.disabled = false;
            }
        }
    }

    function init() {
        initFrame();
        initSSE();
        initSystemStats();
        els.form.addEventListener('submit', startPipeline);
        els.stopBtn.addEventListener('click', stopPipeline);
    }

    init();
})();
