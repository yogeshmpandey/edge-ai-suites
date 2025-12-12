(function () {
    const cfg = window.RUNTIME_CONFIG || {};
    const els = {
        statusDot: document.getElementById('videoStatus'),
        statusText: document.getElementById('statusText'),
        watcherStatus: document.getElementById('watcherStatus'),
        hintEl: document.getElementById('hint'),
        form: document.getElementById('pipelineForm'),
        promptInput: document.getElementById('promptInput'),
        modelNameInput: document.getElementById('modelNameInput'),
        maxTokensInput: document.getElementById('maxTokensInput'),
        rtspInput: document.getElementById('rtspInput'),
        startBtn: document.getElementById('startBtn'),
        stopBtn: document.getElementById('stopBtn'),
        pipelineInfo: document.getElementById('pipelineInfo'),
        runsContainer: document.getElementById('runsContainer'),
    };

    const state = { selectedRunId: null, runs: new Map() };
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
                        ticks: {
                            color: 'rgba(255,255,255,0.55)',
                        },
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

    function extractMetrics(raw) {
        try {
            const payload = JSON.parse(raw);
            const metrics = payload.metrics || {};
            const throughput = metrics.throughput_mean;
            const timestampText =
                payload.timestamp_seconds !== undefined
                    ? `Updated ${payload.timestamp_seconds.toFixed(2)}s into stream`
                    : payload.timestamp
                    ? `Updated at ${new Date(payload.timestamp).toLocaleTimeString()}`
                    : '—';
            return {
                ttft: metrics.ttft_mean ? `${metrics.ttft_mean.toFixed(0)} ms` : '—',
                tpot: metrics.tpot_mean ? `${metrics.tpot_mean.toFixed(2)} ms` : '—',
                throughput: throughput ? `${throughput.toFixed(2)} tok/s` : '—',
                timestamp: timestampText,
            };
        } catch (_err) {
            return { ttft: '—', tpot: '—', throughput: '—', timestamp: '—' };
        }
    }

    function updateWatcher(text) {
        if (els.watcherStatus) els.watcherStatus.textContent = text;
    }

    function updatePipelineInfo(text) {
        els.pipelineInfo.textContent = text;
    }

    function createRunElement(run) {
        const wrap = document.createElement('div');
        wrap.className = 'card';
        wrap.style.marginTop = '10px';
        wrap.style.background = 'var(--panel-strong)';

        const header = document.createElement('div');
        header.className = 'status';
        header.style.margin = '0 0 10px 0';
        header.style.justifyContent = 'space-between';
        header.style.gap = '12px';

        const headerLeft = document.createElement('div');
        headerLeft.style.display = 'flex';
        headerLeft.style.alignItems = 'center';
        headerLeft.style.gap = '8px';
        headerLeft.innerHTML = `
            <span class="dot active"></span>
            <span>Run <strong>${run.runId}</strong> — Stream <strong>${run.peerId}</strong></span>
        `;

        const grid = document.createElement('div');
        grid.style.display = 'flex';
        grid.style.flexDirection = 'column';
        grid.style.gap = '12px';

        const video = document.createElement('iframe');
        video.className = 'run-video';
        video.title = `WebRTC ${run.peerId}`;
        video.style.border = '0';

        const captionPanel = document.createElement('div');
        captionPanel.className = 'caption-panel';
        captionPanel.style.padding = '0';

        const chips = document.createElement('div');
        chips.className = 'chips';
        chips.style.marginTop = '0';
        chips.style.marginBottom = '0';
        chips.innerHTML = `
            <span class="chip"><strong>TTFT</strong><span data-ttft>—</span></span>
            <span class="chip"><strong>TPOT</strong><span data-tpot>—</span></span>
            <span class="chip"><strong>Throughput</strong><span data-throughput>—</span></span>
        `;

        const watcher = document.createElement('div');
        watcher.className = 'status';
        watcher.textContent = 'Waiting for metadata...';

        const caption = document.createElement('p');
        caption.className = 'caption-text';
        caption.textContent = 'Waiting for metadata...';

        const timestamp = document.createElement('div');
        timestamp.className = 'timestamp';
        timestamp.textContent = '—';

        const stopBtn = document.createElement('button');
        stopBtn.className = 'btn btn-danger';
        stopBtn.type = 'button';
        stopBtn.textContent = 'Stop';
        stopBtn.addEventListener('click', async () => {
            stopBtn.disabled = true;
            try {
                await fetch(`/api/runs/${run.runId}`, { method: 'DELETE' });
            } finally {
                const current = state.runs.get(run.runId);
                if (current?.source) current.source.close();
                state.runs.delete(run.runId);
                wrap.remove();
                updatePipelineInfo('Pipeline stopped');
            }
        });

        header.appendChild(headerLeft);
        header.appendChild(stopBtn);

        captionPanel.appendChild(chips);
        captionPanel.appendChild(watcher);
        captionPanel.appendChild(caption);
        captionPanel.appendChild(timestamp);

        grid.appendChild(video);
        grid.appendChild(captionPanel);

        wrap.appendChild(header);
        wrap.appendChild(grid);

        return { wrap, video, caption, watcher, timestamp, chips, stopBtn };
    }

    function attachRunStreams(run, ui) {
        const base = resolveSignalingBase(cfg.signalingUrl);
        if (base) {
            ui.video.src = `${base}/${run.peerId}`;
        }

        const source = new EventSource(`/api/runs/${run.runId}/metadata-stream`);
        source.onmessage = (event) => {
            if (!event.data) {
                ui.watcher.textContent = 'No data yet';
                return;
            }
            setVideoStatus(true, 'Streaming...');
            updateWatcher('Receiving data...');
            ui.caption.textContent = renderCaption(event.data);
            const m = extractMetrics(event.data);
            ui.chips.querySelector('[data-ttft]').textContent = m.ttft;
            ui.chips.querySelector('[data-tpot]').textContent = m.tpot;
            ui.chips.querySelector('[data-throughput]').textContent = m.throughput;
            ui.timestamp.textContent = m.timestamp;
            ui.watcher.textContent = 'Receiving data...';
            if (els.hintEl) els.hintEl.textContent = '';
        };
        source.onerror = () => {
            ui.watcher.textContent = 'Reconnecting...';
            if (els.hintEl) els.hintEl.textContent = 'Stream not found, retrying...';
        };
        state.runs.set(run.runId, { ...run, source });
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
        const prompt = (els.promptInput.value || '').trim() || 'Describe what you see in the image in one sentence.';
        const modelName = (els.modelNameInput?.value || '').trim() || 'OpenGVLab/InternVL2-2B';
        const maxTokensRaw = (els.maxTokensInput?.value || '').toString().trim();
        const maxTokensParsed = Number.parseInt(maxTokensRaw, 10);
        const maxTokens = Number.isFinite(maxTokensParsed) && maxTokensParsed > 0 ? maxTokensParsed : 70;
        if (!rtspUrl) return;
        els.startBtn.disabled = true;
        updatePipelineInfo('Starting pipeline...');
        try {
            const resp = await fetch('/api/runs', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ rtspUrl, prompt, modelName, maxNewTokens: maxTokens })
            });
            const data = await resp.json().catch(async () => ({ message: await resp.text() }));
            if (!resp.ok) throw new Error(data?.message || data?.detail?.message || resp.statusText);

            const run = {
                runId: data.runId,
                pipelineId: data.pipelineId,
                peerId: data.peerId,
                metadataFile: data.metadataFile,
            };

            const ui = createRunElement(run);
            els.runsContainer.prepend(ui.wrap);
            attachRunStreams(run, ui);
            updatePipelineInfo(`Running: ${run.runId} (Pipeline ${run.pipelineId})`);
            els.stopBtn.disabled = true;
        } catch (err) {
            updatePipelineInfo(`Start failed: ${err.message}`);
        } finally {
            els.startBtn.disabled = false;
        }
    }

    async function stopPipeline() {
        updatePipelineInfo('Use Stop on a run card');
    }

    function init() {
        initSystemStats();
        els.form.addEventListener('submit', startPipeline);
        els.stopBtn.addEventListener('click', stopPipeline);
    }

    init();
})();
