/**
 * Metadata stream service for SSE handling
 */
const MetadataStreamService = (function() {
    let metadataSource = null;
    const runUIs = new Map();
    const lastCaptionTime = new Map();

    function initMultiplexedMetadataStream(cfg) {
        // Single SSE connection for all run metadata to avoid browser connection limits
        if (metadataSource) {
            return; // Already initialized
        }

        console.log('Initializing multiplexed metadata stream...');
        metadataSource = new EventSource('/api/runs/metadata-stream');

        metadataSource.onopen = () => {
            console.log('Multiplexed metadata stream connected');
        };

        metadataSource.onmessage = (event) => {
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
                const ui = runUIs.get(runId);
                if (!ui) {
                    console.log(`No UI found for run ${runId}, ignoring metadata`);
                    return; // No UI for this run yet
                }

                // msg.data is already parsed; extract caption and metrics directly
                const data = msg.data;
                const captionText = typeof data === 'object' && data.result ? data.result : (typeof data === 'string' ? data : JSON.stringify(data));
                ui.caption.textContent = captionText;

                // Alert Mode: Apply per-run configurable substring-to-color rules
                if (cfg && cfg.alertMode) {
                    const runCard = ui.wrap;
                    const captionPanel = ui.captionPanel;
                    const lowerCaption = captionText ? captionText.toLowerCase() : '';

                    // Clear inline color state
                    for (const el of [runCard, captionPanel]) {
                        if (!el) continue;
                        el.classList.remove('alert-1', 'alert-2', 'alert-3');
                        el.style.removeProperty('--alert-color');
                        el.style.removeProperty('--alert-color-rgb');
                    }

                    // Use per-run configured rules (no defaults — empty means no alerts)
                    const rules = (ui.alertRules && ui.alertRules.length > 0) ? ui.alertRules : [];

                    // Apply first matching rule (alert-1, alert-2, alert-3 based on rule index)
                    for (let i = 0; i < rules.length; i++) {
                        const rule = rules[i];
                        if (!rule.substring) continue;
                        if (lowerCaption.includes(rule.substring.toLowerCase())) {
                            const hex = rule.color || '#ff4444';
                            const rgb = hexToRgb(hex);
                            const alertClass = 'alert-' + (i + 1);
                            for (const el of [runCard, captionPanel]) {
                                if (!el) continue;
                                el.style.setProperty('--alert-color', hex);
                                if (rgb) el.style.setProperty('--alert-color-rgb', rgb);
                                el.classList.add(alertClass);
                            }
                            break;
                        }
                    }
                }

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

                // Calculate lag: time since this caption was received by the browser
                // We use the browser's own timestamp to avoid clock sync issues with the container
                const receivedAtMs = Date.now();
                // Store when this run last received a caption update
                lastCaptionTime.set(runId, receivedAtMs);
                ui.chips.querySelector('[data-lag]').textContent = '0.00s';

                ui.timestamp.textContent = timestampText;

                console.log(`Updated metadata for run ${runId}`);
            } catch (err) {
                console.error('Error parsing metadata:', err, 'Event data:', event.data);
            }
        };

        metadataSource.onerror = (event) => {
            console.error('Metadata stream error:', event);
            // EventSource will automatically try to reconnect
            // Reset the connection after a delay if it keeps failing
            setTimeout(() => {
                if (metadataSource && metadataSource.readyState === EventSource.CLOSED) {
                    console.log('Reconnecting metadata stream...');
                    metadataSource = null;
                    initMultiplexedMetadataStream(cfg);
                }
            }, 5000);
        };

        metadataSource.onclose = () => {
            console.log('Metadata stream closed');
            metadataSource = null;
        };
    }

    function registerRunUI(runId, ui) {
        runUIs.set(runId, ui);
    }

    function unregisterRunUI(runId) {
        runUIs.delete(runId);
        lastCaptionTime.delete(runId);
    }

    function getLastCaptionTime(runId) {
        return lastCaptionTime.get(runId);
    }

    function hexToRgb(hex) {
        const m = /^#([0-9a-f]{3,6})$/i.exec(hex.trim());
        if (!m) return null;
        let h = m[1];
        if (h.length === 3) h = h[0]+h[0]+h[1]+h[1]+h[2]+h[2];
        const n = parseInt(h, 16);
        return `${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}`;
    }

    function getRunUIs() {
        return runUIs;
    }

    function close() {
        if (metadataSource) {
            console.log('Closing metadata stream');
            metadataSource.close();
            metadataSource = null;
        }
    }

    return {
        initMultiplexedMetadataStream,
        registerRunUI,
        unregisterRunUI,
        getLastCaptionTime,
        getRunUIs,
        close
    };
})();
