(function () {
    const cfg = window.RUNTIME_CONFIG || {};
    const els = {
        statusDot: document.getElementById('videoStatus'),
        hintEl: document.getElementById('hint'),
        form: document.getElementById('pipelineForm'),
        promptInput: document.getElementById('promptInput'),
        modelNameSelect: document.getElementById('modelNameSelect'),
        pipelineSelect: document.getElementById('pipelineSelect'),
        maxTokensInput: document.getElementById('maxTokensInput'),
        rtspInput: document.getElementById('rtspInput'),
        startBtn: document.getElementById('startBtn'),
        pipelineInfo: document.getElementById('pipelineInfo'),
        runsContainer: document.getElementById('runsContainer'),
        themeToggle: document.getElementById('themeToggle'),
    };

    const state = { selectedRunId: null, runs: new Map(), metadataSource: null, runUIs: new Map() };
    const DEFAULT_MODEL = 'InternVL2-1B';
    const DEFAULT_PIPELINE = 'GenAI Pipeline on CPU';
    const THEME_KEY = 'lvc-theme';
    const SETTINGS_KEY = 'lvc-settings';
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

    function updateChartColors() {
        const colors = getChartColors();
        Object.values(statsCharts).forEach(chart => {
            if (chart && chart.options && chart.options.scales && chart.options.scales.y) {
                chart.options.scales.y.grid.color = colors.gridColor;
                chart.options.scales.y.ticks.color = colors.tickColor;
                chart.update('none');
            }
        });
    }

    function toggleTheme() {
        const current = document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark';
        applyTheme(current === 'light' ? 'dark' : 'light');
        updateChartColors();
    }

    function saveSettings() {
        try {
            const settings = {
                rtspUrl: els.rtspInput?.value || '',
                prompt: els.promptInput?.value || '',
                modelName: els.modelNameSelect?.value || '',
                pipelineName: els.pipelineSelect?.value || '',
                maxTokens: els.maxTokensInput?.value || '70',
            };
            localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings));
        } catch (_err) {
            // localStorage not available
        }
    }

    function loadSettings() {
        try {
            const saved = localStorage.getItem(SETTINGS_KEY);
            if (!saved) return null;
            return JSON.parse(saved);
        } catch (_err) {
            return null;
        }
    }

    function restoreSettings() {
        const settings = loadSettings();
        if (!settings) return;
        
        if (settings.rtspUrl && els.rtspInput) {
            els.rtspInput.value = settings.rtspUrl;
        }
        if (settings.prompt && els.promptInput) {
            els.promptInput.value = settings.prompt;
        }
        if (settings.maxTokens && els.maxTokensInput) {
            els.maxTokensInput.value = settings.maxTokens;
        }
        // Model and pipeline will be restored after options are loaded
    }

    function restoreSelectValues() {
        const settings = loadSettings();
        if (!settings) return;
        
        if (settings.modelName && els.modelNameSelect) {
            const options = Array.from(els.modelNameSelect.options).map(o => o.value);
            if (options.includes(settings.modelName)) {
                els.modelNameSelect.value = settings.modelName;
            }
        }
        if (settings.pipelineName && els.pipelineSelect) {
            const options = Array.from(els.pipelineSelect.options).map(o => o.value);
            if (options.includes(settings.pipelineName)) {
                els.pipelineSelect.value = settings.pipelineName;
            }
        }
    }

    function setupSettingsPersistence() {
        // Save settings on input changes
        const inputs = [els.rtspInput, els.promptInput, els.maxTokensInput, els.modelNameSelect, els.pipelineSelect];
        inputs.forEach(el => {
            if (el) {
                el.addEventListener('change', saveSettings);
                el.addEventListener('input', saveSettings);
            }
        });
    }

    function getChartColors() {
        const isLight = document.documentElement.getAttribute('data-theme') === 'light';
        return {
            gridColor: isLight ? 'rgba(0,0,0,0.08)' : 'rgba(255,255,255,0.05)',
            tickColor: isLight ? 'rgba(0,0,0,0.65)' : 'rgba(255,255,255,0.55)',
        };
    }

    function createStatChart(elId, label, color) {
        const ctx = document.getElementById(elId)?.getContext('2d');
        if (!ctx) return null;
        const gradient = ctx.createLinearGradient(0, 0, 0, 140);
        gradient.addColorStop(0, `${color}55`);
        gradient.addColorStop(1, `${color}0f`);
        const colors = getChartColors();
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
                        grid: { color: colors.gridColor },
                        ticks: {
                            color: colors.tickColor,
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
            restoreSelectValues();
            updatePipelineInfo('Models loaded');
        } catch (_err) {
            setModelOptions([DEFAULT_MODEL]);
            restoreSelectValues();
            updatePipelineInfo('Model list unavailable, using default');
        }
    }

       async function loadPipelines() {
           try {
               const resp = await fetch('/api/pipelines');
               if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
               const data = await resp.json();
               setPipelineOptions(data?.pipelines);
               restoreSelectValues();
           } catch (_err) {
               setPipelineOptions([DEFAULT_PIPELINE]);
               restoreSelectValues();
           }
       }

    function tearDownRun(runId, current, message) {
        console.log(`Tearing down run ${runId}`);
        // Remove UI reference from multiplexed stream handler
        state.runUIs.delete(runId);
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
                if ((resp.status === 404) || (resp.status === 502)) {
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
            if (msg.includes('404') || msg.includes('not found') || msg.includes('502')) {
                tearDownRun(runId, current, 'Run missing on server, removing');
            } else {
                // Re-enable the stop button so user can retry
                if (current.stopBtn) {
                    current.stopBtn.disabled = false;
                    current.stopBtn.textContent = 'Stop';
                }
                updatePipelineInfo(`Stop failed: ${err.message}`);
                console.error('Stop run error:', err);
            }
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
        headerLeft.style.flexWrap = 'wrap';
        
        // Determine device from pipeline name
        const deviceType = (run.pipelineName || '').toLowerCase().includes('gpu') ? 'GPU' : 'CPU';
        const deviceIcon = deviceType === 'GPU' 
            ? '<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><line x1="9" y1="9" x2="15" y2="9"/><line x1="9" y1="12" x2="15" y2="12"/><line x1="9" y1="15" x2="15" y2="15"/></svg>'
            : '<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><circle cx="12" cy="12" r="3"/></svg>';
        
        headerLeft.innerHTML = `
            <span class="dot active"></span>
            <span>Run <strong>${run.runId}</strong></span>
            <span style="color: var(--muted); margin: 0 4px;">|</span>
            <span class="chip" style="background: var(--accent); color: var(--bg);">
                ${deviceIcon}
                <strong>${deviceType}</strong>
            </span>
            <span class="chip">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2L2 7l10 5 10-5-10-5z"/><path d="M2 17l10 5 10-5"/><path d="M2 12l10 5 10-5"/></svg>
                ${run.modelName || 'Unknown'}
            </span>
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
        stopBtn.addEventListener('click', async (e) => {
            e.preventDefault();
            e.stopPropagation();
            if (stopBtn.disabled) return;
            stopBtn.disabled = true;
            stopBtn.textContent = 'Stopping...';
            await stopRun(run.runId);
            // Note: stopRun handles all error cases internally and resets button state on failure
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

    function initMultiplexedMetadataStream() {
        // Single SSE connection for all run metadata to avoid browser connection limits
        if (state.metadataSource) {
            return; // Already initialized
        }
        
        console.log('Initializing multiplexed metadata stream...');
        state.metadataSource = new EventSource('/api/runs/metadata-stream');
        
        state.metadataSource.onopen = () => {
            console.log('Multiplexed metadata stream connected');
        };
        
        state.metadataSource.onmessage = (event) => {
            if (!event.data) return;
            
            try {
                const msg = JSON.parse(event.data);
                const runId = msg.runId;
                
                if (!runId) {
                    console.warn('Received metadata without runId:', msg);
                    return;
                }
                
                // Handle run removal notification
                if (msg.removed) {
                    console.log(`Run ${runId} removed from server`);
                    return;
                }
                
                // Get the UI elements for this run
                const ui = state.runUIs.get(runId);
                if (!ui) {
                    console.log(`No UI found for run ${runId}, ignoring metadata`);
                    return; // No UI for this run yet
                }
                
                // msg.data is already parsed; extract caption and metrics directly
                const data = msg.data;
                const captionText = typeof data === 'object' && data.result ? data.result : (typeof data === 'string' ? data : JSON.stringify(data));
                ui.caption.textContent = captionText;
                
                // Extract metrics from the data object
                const metrics = (typeof data === 'object' && data.metrics) ? data.metrics : {};
                const throughput = metrics.throughput_mean;
                const timestampText =
                    data.timestamp_seconds !== undefined
                        ? `Updated ${data.timestamp_seconds.toFixed(2)}s into stream`
                        : data.timestamp
                        ? `Updated at ${new Date(data.timestamp).toLocaleTimeString()}`
                        : '—';
                ui.chips.querySelector('[data-ttft]').textContent = metrics.ttft_mean ? `${metrics.ttft_mean.toFixed(0)} ms` : '—';
                ui.chips.querySelector('[data-tpot]').textContent = metrics.tpot_mean ? `${metrics.tpot_mean.toFixed(2)} ms` : '—';
                ui.chips.querySelector('[data-throughput]').textContent = throughput ? `${throughput.toFixed(2)} tok/s` : '—';
                ui.timestamp.textContent = timestampText;
                if (els.hintEl) els.hintEl.textContent = '';
                
                console.log(`Updated metadata for run ${runId}`);
            } catch (err) {
                console.error('Error parsing metadata:', err, 'Event data:', event.data);
            }
        };
        
        state.metadataSource.onerror = (event) => {
            console.error('Metadata stream error:', event);
            // EventSource will automatically try to reconnect
            // Reset the connection after a delay if it keeps failing
            setTimeout(() => {
                if (state.metadataSource && state.metadataSource.readyState === EventSource.CLOSED) {
                    console.log('Reconnecting metadata stream...');
                    state.metadataSource = null;
                    initMultiplexedMetadataStream();
                }
            }, 5000);
        };
        
        state.metadataSource.onclose = () => {
            console.log('Metadata stream closed');
            state.metadataSource = null;
        };
    }

    function attachRunStreams(run, ui) {
        const base = resolveSignalingBase(cfg.signalingUrl);
        if (base) {
            ui.video.src = `${base}/${run.peerId}`;
        }

        // Store UI reference for the multiplexed metadata stream
        state.runUIs.set(run.runId, ui);
        
        // Initialize the multiplexed stream if not already done
        initMultiplexedMetadataStream();
        
        // Store run info without individual EventSource
        state.runs.set(run.runId, { ...run, ui });
        // Keep references for UI teardown
        state.runs.get(run.runId).wrap = ui.wrap;
        state.runs.get(run.runId).stopBtn = ui.stopBtn;
    }

    async function restoreActiveRuns() {
        // Fetch active runs from backend and restore UI cards
        try {
            const resp = await fetch('/api/runs');
            if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
            const runs = await resp.json();
            
            if (runs.length === 0) {
                return;
            }
            
            // Hide hint if there are active runs
            if (els.hintEl) els.hintEl.style.display = 'none';
            
            for (const runData of runs) {
                const run = {
                    runId: runData.runId,
                    pipelineId: runData.pipelineId,
                    peerId: runData.peerId,
                    metadataFile: runData.metadataFile,
                    modelName: runData.modelName || 'Unknown',
                    pipelineName: runData.pipelineName || '',
                };
                
                const ui = createRunElement(run);
                els.runsContainer.appendChild(ui.wrap);
                attachRunStreams(run, ui);
                state.selectedRunId = run.runId;
            }
            
            updatePipelineInfo(`Restored ${runs.length} active run(s)`);
        } catch (err) {
            console.warn('Failed to restore active runs:', err);
        }
    }

    function initCollectorMetrics() {
        // Initialize charts
        statsCharts.cpu = createStatChart('cpuChart', 'CPU %', '#1ad0ff');
        statsCharts.ram = createStatChart('ramChart', 'RAM %', '#8ca0c2');
        statsCharts.gpu = createStatChart('gpuChart', 'GPU %', '#ffb347');

        // DOM elements for metrics display
        const cpuVal = document.getElementById('cpuVal');
        const ramVal = document.getElementById('ramVal');
        const gpuVal = document.getElementById('gpuVal');
        const gpuDetail = document.getElementById('gpuDetail');
        const gpuEngines = document.getElementById('gpuEngines');
        const gpuFreq = document.getElementById('gpuFreq');
        const gpuPower = document.getElementById('gpuPower');
        const gpuTemp = document.getElementById('gpuTemp');
        const gpuError = document.getElementById('gpuError');

        // WebSocket connection to metrics collector
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws/clients`;
        
        let metricsWS = null;
        let reconnectTimeout = null;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 10;
        const reconnectDelay = 3000;

        // Data structures for tracking GPU engine metrics
        const gpuEngineData = {};
        // Track GPU power values separately for combining
        let gpuPowerValue = null;
        let pkgPowerValue = null;

        function connectMetricsWS() {
            if (metricsWS && (metricsWS.readyState === WebSocket.CONNECTING || metricsWS.readyState === WebSocket.OPEN)) {
                console.log('WebSocket already connected or connecting');
                return;
            }

            console.log('Connecting to metrics collector WebSocket:', wsUrl);
            metricsWS = new WebSocket(wsUrl);

            metricsWS.onopen = () => {
                console.log('Metrics WebSocket connected');
                reconnectAttempts = 0;
                
                // Update UI to show collector connected
                const collectorStatus = document.getElementById('collectorStatus');
                const collectorStatusDot = document.getElementById('collectorStatusDot');
                if (collectorStatus) {
                    collectorStatus.textContent = 'Connected';
                    collectorStatus.className = 'status-connected';
                }
                if (collectorStatusDot) {
                    collectorStatusDot.classList.add('active');
                }
            };

            metricsWS.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (!data.metrics || !Array.isArray(data.metrics)) {
                        return;
                    }

                    processCollectorMetrics(data.metrics);
                } catch (err) {
                    console.error('Error parsing metrics message:', err);
                }
            };

            metricsWS.onerror = (error) => {
                console.error('Metrics WebSocket error:', error);
            };

            metricsWS.onclose = () => {
                console.log('Metrics WebSocket closed');
                
                // Update UI to show collector disconnected
                const collectorStatus = document.getElementById('collectorStatus');
                const collectorStatusDot = document.getElementById('collectorStatusDot');
                if (collectorStatus) {
                    collectorStatus.textContent = 'Disconnected';
                    collectorStatus.className = 'status-disconnected';
                }
                if (collectorStatusDot) {
                    collectorStatusDot.classList.remove('active');
                }

                // Attempt to reconnect
                if (reconnectAttempts < maxReconnectAttempts) {
                    reconnectAttempts++;
                    console.log(`Attempting to reconnect (${reconnectAttempts}/${maxReconnectAttempts})...`);
                    reconnectTimeout = setTimeout(connectMetricsWS, reconnectDelay);
                } else {
                    console.error('Max reconnect attempts reached for metrics WebSocket');
                }
            };
        }

        function processCollectorMetrics(metrics) {
            // Reset power values for this batch
            gpuPowerValue = null;
            pkgPowerValue = null;
            
            metrics.forEach(metric => {
                const { name, fields, tags } = metric;
                
                switch (name) {
                    case 'cpu':
                        if (fields.usage_user !== undefined) {
                            const cpuUsage = fields.usage_user;
                            pushStatSample('cpu', cpuUsage);
                            
                            if (cpuVal) {
                                cpuVal.textContent = `${cpuUsage.toFixed(1)}%`;
                            }
                        }
                        break;
                    
                    case 'mem':
                        if (fields.used_percent !== undefined) {
                            const memUsage = fields.used_percent;
                            pushStatSample('ram', memUsage);
                            
                            if (ramVal) {
                                ramVal.textContent = `${memUsage.toFixed(1)}%`;
                            }
                        }
                        break;
                    
                    case 'gpu_engine_usage':
                        if (fields.usage !== undefined && tags.engine) {
                            const engineName = tags.engine.toUpperCase();
                            const usage = fields.usage;
                            
                            // Store engine data
                            gpuEngineData[engineName] = usage;
                        }
                        break;
                    
                    case 'gpu_frequency':
                        if (fields.value !== undefined && tags.type === 'cur_freq') {
                            const freqMHz = fields.value;
                            if (gpuFreq) {
                                gpuFreq.textContent = `Freq: ${freqMHz} MHz`;
                                gpuFreq.style.display = 'block';
                            }
                        }
                        break;
                    
                    case 'gpu_power':
                        if (fields.value !== undefined) {
                            const powerType = tags.type;
                            const powerW = fields.value;
                            
                            if (powerType === 'gpu_cur_power') {
                                gpuPowerValue = powerW;
                            } else if (powerType === 'pkg_cur_power') {
                                pkgPowerValue = powerW;
                            }
                        }
                        break;
                    
                    case 'temp':
                        if (fields.temp !== undefined) {
                            const tempC = fields.temp;
                            const sensor = tags.sensor || 'unknown';
                            
                            // Display package temperature
                            if (gpuTemp && sensor.includes('package')) {
                                gpuTemp.textContent = `Temp: ${tempC}°C`;
                                gpuTemp.style.display = 'block';
                            }
                        }
                        break;
                    
                    case 'cpu_frequency_avg':
                        // CPU frequency - could be displayed if needed
                        break;
                    
                    case 'fps':
                        // FPS metrics - could be displayed if needed
                        break;
                }
            });
            
            // Update GPU power display after processing all metrics
            if (gpuPower && gpuPowerValue !== null) {
                let powerText = `Power: ${gpuPowerValue.toFixed(1)}W`;
                if (pkgPowerValue !== null) {
                    powerText += ` (Pkg: ${pkgPowerValue.toFixed(1)}W)`;
                }
                gpuPower.textContent = powerText;
                gpuPower.style.display = 'block';
            }
            
            // Update GPU engines display
            const engineNames = Object.keys(gpuEngineData);
            if (gpuEngines && engineNames.length > 0) {
                const engineList = engineNames
                    .map(name => `${formatEngineName(name)}: ${gpuEngineData[name].toFixed(1)}%`)
                    .join(' | ');
                gpuEngines.textContent = engineList;
                gpuEngines.style.display = 'block';
            }
            
            // Calculate overall GPU usage from engines
            const engineMetrics = metrics.filter(m => m.name === 'gpu_engine_usage');
            if (engineMetrics.length > 0) {
                // Use max engine usage as GPU usage (typically RCS is the main compute engine)
                const maxGpuUsage = Math.max(...engineMetrics.map(m => m.fields.usage || 0));
                pushStatSample('gpu', maxGpuUsage);
                
                if (gpuVal) {
                    gpuVal.textContent = `${maxGpuUsage.toFixed(1)}%`;
                }
                
                // Mark GPU as available
                if (gpuDetail) gpuDetail.style.display = 'block';
                if (gpuError) gpuError.style.display = 'none';
            }
        }

        // Start connection
        connectMetricsWS();
        
        // Cleanup on page unload
        window.addEventListener('beforeunload', () => {
            if (reconnectTimeout) {
                clearTimeout(reconnectTimeout);
            }
            if (metricsWS) {
                metricsWS.close();
            }
        });
    }

    function formatEngineName(name) {
        // Format engine names for display (e.g., "rcs0" -> "RCS0", "video" -> "Video")
        if (!name) return 'Unknown';
        return name.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
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
                modelName: modelName,
                pipelineName: pipelineName,
            };

            // Hide the hint when first pipeline starts
            if (els.hintEl) els.hintEl.style.display = 'none';

            const ui = createRunElement(run);
            els.runsContainer.appendChild(ui.wrap);
            attachRunStreams(run, ui);
            updatePipelineInfo(`Latest Run ID: (${run.runId})`);
            state.selectedRunId = run.runId;
        } catch (err) {
            updatePipelineInfo(`Start failed: ${err.message}`);
        } finally {
            els.startBtn.disabled = false;
        }
    }

    function init() {
        applyTheme(detectInitialTheme());
        if (els.themeToggle) els.themeToggle.addEventListener('click', toggleTheme);
        
        // Restore settings from localStorage before loading options
        restoreSettings();
        setupSettingsPersistence();
        
        loadModels();
        loadPipelines();
        initCollectorMetrics(); // Initialize WebSocket metrics from collector
        
        // Restore active runs from backend
        restoreActiveRuns();
        
        els.form.addEventListener('submit', startPipeline);
        
        // Cleanup SSE connections when page unloads
        window.addEventListener('beforeunload', () => {
            if (state.metadataSource) {
                console.log('Closing metadata stream on page unload');
                state.metadataSource.close();
                state.metadataSource = null;
            }
        });
    }

    init();
})();