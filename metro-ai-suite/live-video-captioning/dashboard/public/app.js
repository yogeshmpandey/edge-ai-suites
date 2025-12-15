(function () {
    const cfg = window.RUNTIME_CONFIG || {};
    const els = {
        statusDot: document.getElementById('videoStatus'),
        watcherStatus: document.getElementById('watcherStatus'),
        hintEl: document.getElementById('hint'),
        form: document.getElementById('pipelineForm'),
        promptInput: document.getElementById('promptInput'),
        modelNameSelect: document.getElementById('modelNameSelect'),
        pipelineSelect: document.getElementById('pipelineSelect'),
        maxTokensInput: document.getElementById('maxTokensInput'),
        rtspInput: document.getElementById('rtspInput'),
        startBtn: document.getElementById('startBtn'),
        stopBtn: document.getElementById('stopBtn'),
        pipelineInfo: document.getElementById('pipelineInfo'),
        runsContainer: document.getElementById('runsContainer'),
        themeToggle: document.getElementById('themeToggle'),
    };

    const state = { selectedRunId: null, runs: new Map() };
    const DEFAULT_MODEL = 'InternVL2-1B';
    const DEFAULT_PIPELINE = 'GenAI Pipeline on CPU';
    const THEME_KEY = 'lvc-theme';
    const statsCharts = {};

    function detectInitialTheme() {
        try {
            const saved = localStorage.getItem(THEME_KEY);
            if (saved === 'light' || saved === 'dark') return saved;
        } catch (_err) {}
        if (window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches) return 'light';
        return 'dark';
    }

    function applyTheme(theme) {
        const next = theme === 'light' ? 'light' : 'dark';
        document.documentElement.setAttribute('data-theme', next);
        if (els.themeToggle) {
            els.themeToggle.setAttribute('aria-label', next === 'light' ? 'Switch to dark mode' : 'Switch to light mode');
        }
        try {
            localStorage.setItem(THEME_KEY, next);
        } catch (_err) {}
    }

    function toggleTheme() {
        const current = document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark';
        applyTheme(current === 'light' ? 'dark' : 'light');
    }

    function refreshGlobalStopButton() {
        if (!els.stopBtn) return;
        els.stopBtn.disabled = state.runs.size === 0;
    }

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

    function updatePipelineInfo(text) {
        els.pipelineInfo.textContent = text;
    }

    function setModelOptions(models) {
        const select = els.modelNameSelect;
        if (!select) return;
        select.innerHTML = '';
        const list = Array.isArray(models) && models.length ? models : [DEFAULT_MODEL];
        for (const name of list) {
            const opt = document.createElement('option');
            opt.value = name;
            opt.textContent = name;
            select.appendChild(opt);
        }
        const preferred = list.includes(DEFAULT_MODEL) ? DEFAULT_MODEL : list[0];
        select.value = preferred;
    }

       function setPipelineOptions(pipelines) {
           const select = els.pipelineSelect;
           if (!select) return;
           select.innerHTML = '';
           const list = Array.isArray(pipelines) && pipelines.length ? pipelines : [DEFAULT_PIPELINE];
           for (const name of list) {
               const opt = document.createElement('option');
               opt.value = name;
               opt.textContent = name;
               select.appendChild(opt);
           }
           select.value = list[0];
       }

    async function loadModels() {
        try {
            const resp = await fetch('/api/models');
            if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
            const data = await resp.json();
            setModelOptions(data?.models);
            updatePipelineInfo('Models loaded');
        } catch (_err) {
            setModelOptions([DEFAULT_MODEL]);
            updatePipelineInfo('Model list unavailable, using default');
        }
    }

       async function loadPipelines() {
           try {
               const resp = await fetch('/api/pipelines');
               if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
               const data = await resp.json();
               setPipelineOptions(data?.pipelines);
           } catch (_err) {
               setPipelineOptions([DEFAULT_PIPELINE]);
           }
       }

    function tearDownRun(runId, current, message) {
        if (current?.source) current.source.close();
        if (current?.wrap) current.wrap.remove();
        state.runs.delete(runId);
        if (state.selectedRunId === runId) state.selectedRunId = null;
        if (message) updatePipelineInfo(message);
        // Show hint again when all runs are stopped
        if (state.runs.size === 0 && els.hintEl) {
            els.hintEl.style.display = 'block';
            els.hintEl.textContent = 'Start a pipeline to see video streams here';
        }
    }

    async function stopRun(runId) {
        const current = state.runs.get(runId);
        if (!current) return;

        updatePipelineInfo(`Stopping: ${runId}...`);
        try {
            const resp = await fetch(`/api/runs/${runId}`, { method: 'DELETE' });
            if (!resp.ok) {
                if (resp.status === 404) {
                    // Backend no longer tracks the run; clean up UI card anyway.
                    tearDownRun(runId, current, 'Run missing on server, removing');
                    return;
                }
                const data = await resp.json().catch(async () => ({ message: await resp.text() }));
                throw new Error(data?.message || data?.detail?.message || resp.statusText);
            }

            tearDownRun(runId, current, state.runs.size <= 1 ? 'Pipeline stopped' : `Stopped: ${runId}`);
        } catch (err) {
            const msg = (err?.message || '').toLowerCase();
            if (msg.includes('404') || msg.includes('not found')) {
                tearDownRun(runId, current, 'Run missing on server, removing');
            } else {
                throw err;
            }
        } finally {
            refreshGlobalStopButton();
        }
    }

    function createRunElement(run) {
        const wrap = document.createElement('div');
        wrap.className = 'card';
        wrap.style.background = 'var(--panel-strong)';

        const header = document.createElement('div');
        header.className = 'status';
        header.style.margin = '0 0 10px 0';
        header.style.justifyContent = 'space-between';
        header.style.gap = '12px';
        header.style.flexWrap = 'wrap';

        const headerLeft = document.createElement('div');
        headerLeft.style.display = 'flex';
        headerLeft.style.alignItems = 'center';
        headerLeft.style.gap = '8px';
        headerLeft.style.fontSize = '0.85rem';
        headerLeft.innerHTML = `
            <span class="dot active"></span>
            <span>Running Pipeline -  <strong>${run.runId}</strong></span>
        `;

        const grid = document.createElement('div');
        grid.style.display = 'flex';
        grid.style.flexDirection = 'column';
        grid.style.gap = '6px';
        grid.style.flex = '1';
        grid.style.minHeight = '0';
        grid.style.overflow = 'hidden';

        const video = document.createElement('iframe');
        video.className = 'run-video';
        video.title = `WebRTC ${run.peerId}`;
        video.style.border = '0';
        video.style.flex = '1';
        video.style.minHeight = '0';

        const captionPanel = document.createElement('div');
        captionPanel.className = 'caption-panel';
        captionPanel.style.padding = '0';
        captionPanel.style.flexShrink = '0';
        captionPanel.style.maxHeight = '100px';
        captionPanel.style.overflow = 'hidden';

        const chips = document.createElement('div');
        chips.className = 'chips';
        chips.style.marginTop = '0';
        chips.style.marginBottom = '0';
        chips.style.fontSize = '0.8rem';
        chips.innerHTML = `
            <span class="chip"><strong>TTFT</strong><span data-ttft>—</span></span>
            <span class="chip"><strong>TPOT</strong><span data-tpot>—</span></span>
            <span class="chip"><strong>Throughput</strong><span data-throughput>—</span></span>
        `;

        const timestamp = document.createElement('div');
        timestamp.className = 'timestamp';
        timestamp.style.fontSize = '0.75rem';
        timestamp.style.marginLeft = 'auto';
        timestamp.style.whiteSpace = 'nowrap';
        timestamp.textContent = '—';

        // Chips row container with chips on left and timestamp on right
        const chipsRow = document.createElement('div');
        chipsRow.style.display = 'flex';
        chipsRow.style.alignItems = 'center';
        chipsRow.style.justifyContent = 'space-between';
        chipsRow.style.gap = '8px';
        chipsRow.appendChild(chips);
        chipsRow.appendChild(timestamp);

        const watcher = document.createElement('div');
        watcher.className = 'status';
        watcher.style.fontSize = '0.8rem';
        watcher.style.marginBottom = '2px';

        const caption = document.createElement('p');
        caption.className = 'caption-text';
        caption.textContent = 'Waiting for metadata...';

        const stopBtn = document.createElement('button');
        stopBtn.className = 'btn btn-danger';
        stopBtn.type = 'button';
        stopBtn.style.fontSize = '0.85rem';
        stopBtn.style.padding = '6px 12px';
        stopBtn.textContent = 'Stop';
        stopBtn.addEventListener('click', async () => {
            stopBtn.disabled = true;
            try {
                await stopRun(run.runId);
            } catch (err) {
                updatePipelineInfo(`Stop failed: ${err.message}`);
                stopBtn.disabled = false;
            }
        });

        header.appendChild(headerLeft);
        header.appendChild(stopBtn);

        captionPanel.appendChild(chipsRow);
        captionPanel.appendChild(watcher);
        captionPanel.appendChild(caption);

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
                return;
            }
            ui.caption.textContent = renderCaption(event.data);
            const m = extractMetrics(event.data);
            ui.chips.querySelector('[data-ttft]').textContent = m.ttft;
            ui.chips.querySelector('[data-tpot]').textContent = m.tpot;
            ui.chips.querySelector('[data-throughput]').textContent = m.throughput;
            ui.timestamp.textContent = m.timestamp;
            if (els.hintEl) els.hintEl.textContent = '';
        };
        source.onerror = () => {
            if (els.hintEl) els.hintEl.textContent = 'Stream not found, retrying...';
        };
        state.runs.set(run.runId, { ...run, source });
        // Keep references so the global Stop button can cleanly teardown UI.
        state.runs.get(run.runId).wrap = ui.wrap;
        state.runs.get(run.runId).stopBtn = ui.stopBtn;
        refreshGlobalStopButton();
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
        const modelName = (els.modelNameSelect?.value || '').trim() || DEFAULT_MODEL;
        const pipelineName = (els.pipelineSelect?.value || '').trim() || DEFAULT_PIPELINE;
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
                   body: JSON.stringify({ rtspUrl, prompt, modelName, maxNewTokens: maxTokens, pipelineName })
            });
            const data = await resp.json().catch(async () => ({ message: await resp.text() }));
            if (!resp.ok) throw new Error(data?.message || data?.detail?.message || resp.statusText);

            const run = {
                runId: data.runId,
                pipelineId: data.pipelineId,
                peerId: data.peerId,
                metadataFile: data.metadataFile,
            };

            // Hide the hint when first pipeline starts
            if (els.hintEl) els.hintEl.style.display = 'none';

            const ui = createRunElement(run);
            els.runsContainer.appendChild(ui.wrap);
            attachRunStreams(run, ui);
            updatePipelineInfo(`Running: ${run.runId} (Pipeline ${run.pipelineId})`);
            state.selectedRunId = run.runId;
            refreshGlobalStopButton();
        } catch (err) {
            updatePipelineInfo(`Start failed: ${err.message}`);
        } finally {
            els.startBtn.disabled = false;
        }
    }

    function init() {
        applyTheme(detectInitialTheme());
        if (els.themeToggle) els.themeToggle.addEventListener('click', toggleTheme);
        loadModels();
           loadPipelines();
        initSystemStats();
        els.form.addEventListener('submit', startPipeline);
        if (els.stopBtn) {
            // Global stop button removed from UI, but keep compatibility if re-added.
            els.stopBtn.addEventListener('click', async () => {
                const preferred = state.selectedRunId;
                const runId = preferred && state.runs.has(preferred) ? preferred : Array.from(state.runs.keys()).pop();
                if (!runId) {
                    updatePipelineInfo('No active runs');
                    refreshGlobalStopButton();
                    return;
                }
                els.stopBtn.disabled = true;
                try {
                    await stopRun(runId);
                } catch (err) {
                    updatePipelineInfo(`Stop failed: ${err.message}`);
                } finally {
                    refreshGlobalStopButton();
                }
            });
            refreshGlobalStopButton();
        }
    }

    init();
})();
