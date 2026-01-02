"""
GPU metrics collector for the Live Video Captioning Dashboard.

Collects GPU metrics from qmassa JSON output file using async I/O.
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import Any, Optional

from src.models.gpu import GPUMetrics

logger = logging.getLogger(__name__)


class QmassaCollector:
    """Collector for GPU metrics from host-generated qmassa JSON file.
    
    Qmassa must be run on the host with sudo to see all GPU clients:
        sudo qmassa -x -t /tmp/qmassa-stats.json -m 1000
    
    The dashboard container reads from this file via the /tmp volume mount.
    
    This collector uses asyncio for non-blocking file reading, replacing
    the threading-based approach for better integration with the async
    event loop.
    """

    def __init__(self, json_file: Path, poll_interval_ms: int = 1000) -> None:
        """Initialize the collector.
        
        Args:
            json_file: Path to the qmassa JSON output file.
            poll_interval_ms: How often to poll the file in milliseconds.
        """
        self._json_file = json_file
        self._poll_interval = poll_interval_ms / 1000.0
        self._latest_metrics: GPUMetrics = GPUMetrics()
        self._task: Optional[asyncio.Task[None]] = None
        self._wait_logged = False

    async def start(self) -> None:
        """Start the background reader task."""
        if self._task is None:
            self._task = asyncio.create_task(self._reader_loop())
            logger.info(f"GPU metrics collector started, polling {self._json_file}")

    async def stop(self) -> None:
        """Stop the background reader task."""
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
            logger.info("GPU metrics collector stopped")

    def get_metrics(self) -> GPUMetrics:
        """Get the latest GPU metrics.
        
        Returns:
            Most recent GPUMetrics snapshot.
        """
        return self._latest_metrics

    async def _reader_loop(self) -> None:
        """Periodically read and parse the qmassa JSON file."""
        while True:
            try:
                if not self._json_file.exists():
                    if not self._wait_logged:
                        self._latest_metrics = GPUMetrics(
                            error="Run on host: sudo qmassa -x -t /tmp/qmassa-stats.json"
                        )
                        self._wait_logged = True
                else:
                    self._latest_metrics = await self._parse_json_file()
                    self._wait_logged = False
            except Exception as e:
                self._latest_metrics = GPUMetrics(error=f"Parse error: {e}")
                logger.exception("Error reading GPU metrics")

            await asyncio.sleep(self._poll_interval)

    async def _parse_json_file(self) -> GPUMetrics:
        """Parse the qmassa JSON output file.
        
        Returns:
            Parsed GPUMetrics from the file.
        """
        try:
            content = await asyncio.to_thread(
                self._json_file.read_text, encoding="utf-8"
            )
            # qmassa writes JSON lines format
            lines = [line.strip() for line in content.strip().split("\n") if line.strip()]
            if not lines:
                return GPUMetrics(error="No data yet")

            # Parse the last line (most recent data)
            data = json.loads(lines[-1])
            return self._extract_metrics(data)

        except json.JSONDecodeError as e:
            return GPUMetrics(error=f"JSON parse error: {e}")
        except OSError as e:
            return GPUMetrics(error=f"Read error: {e}")

    def _extract_metrics(self, data: dict[str, Any]) -> GPUMetrics:
        """Extract GPU metrics from qmassa JSON data.
        
        Args:
            data: Parsed JSON data from qmassa.
            
        Returns:
            Populated GPUMetrics instance.
        """
        metrics = GPUMetrics(available=True)

        # qmassa v1.2.0 format: devs_state array contains GPU devices
        devices = data.get("devs_state", [])
        if not devices:
            devices = data.get("devices", [])
            if not devices and isinstance(data, list):
                devices = data

        if not devices:
            return GPUMetrics(error="No GPU devices found in qmassa output")

        # Filter to supported GPU devices
        gpu_devices = self._filter_gpu_devices(devices)

        if not gpu_devices:
            return GPUMetrics(error="No supported GPU devices found")

        # Use the first GPU device
        dev = gpu_devices[0]
        self._populate_device_info(metrics, dev)
        self._populate_engine_usage(metrics, dev)
        self._populate_memory_stats(metrics, dev)
        self._populate_frequency(metrics, dev)
        self._populate_power(metrics, dev)
        self._populate_temperature(metrics, dev)

        return metrics

    def _filter_gpu_devices(self, devices: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Filter to supported GPU devices.
        
        Args:
            devices: List of device dicts from qmassa.
            
        Returns:
            Filtered list of GPU devices.
        """
        gpu_devices = []
        for dev in devices:
            drv_name = dev.get("drv_name", "")
            dev_type = dev.get("dev_type", "Unknown")
            # Skip ASPEED BMC and unknown devices
            if drv_name in ("ast", ""):
                continue
            # Prefer Intel/AMD GPUs
            if drv_name in ("xe", "i915", "amdgpu") or dev_type in ("Discrete", "Integrated"):
                gpu_devices.append(dev)

        if not gpu_devices:
            # Fallback to any device with engine stats
            gpu_devices = [d for d in devices if d.get("dev_stats", {}).get("eng_usage")]

        return gpu_devices

    def _populate_device_info(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate basic device info."""
        metrics.pci_slot = dev.get("pci_dev", "")
        metrics.device_name = dev.get("vdr_dev", dev.get("pci_id", "Unknown GPU"))
        metrics.driver = dev.get("drv_name", "")
        metrics.device_type = dev.get("dev_type", "Unknown")

    def _populate_engine_usage(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate engine utilization metrics."""
        dev_stats = dev.get("dev_stats", {})
        eng_usage = dev_stats.get("eng_usage", {})
        
        if isinstance(eng_usage, dict):
            for name, usage_list in eng_usage.items():
                if isinstance(usage_list, list) and len(usage_list) > 0:
                    latest = usage_list[-1] if usage_list else 0
                    metrics.engines[name] = float(latest) if latest else 0.0
                elif isinstance(usage_list, (int, float)):
                    metrics.engines[name] = float(usage_list)

        # If device-level is all zeros, aggregate from clients
        if all(v == 0.0 for v in metrics.engines.values()):
            self._aggregate_client_engines(metrics, dev)

        # Calculate total usage
        if metrics.engines:
            metrics.total_engine_percent = sum(metrics.engines.values()) / len(metrics.engines)

    def _aggregate_client_engines(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Aggregate engine usage from client stats."""
        clis_stats = dev.get("clis_stats", [])
        aggregated: dict[str, float] = {}
        
        for cli in clis_stats:
            cli_eng = cli.get("eng_usage", {})
            for name, usage_list in cli_eng.items():
                if isinstance(usage_list, list) and len(usage_list) > 0:
                    latest = usage_list[-1] if usage_list else 0
                    if latest:
                        aggregated[name] = aggregated.get(name, 0) + float(latest)
                elif isinstance(usage_list, (int, float)) and usage_list:
                    aggregated[name] = aggregated.get(name, 0) + float(usage_list)

        if aggregated:
            metrics.engines = aggregated

    def _populate_memory_stats(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate memory statistics."""
        dev_stats = dev.get("dev_stats", {})
        mem_info_list = dev_stats.get("mem_info", [])
        
        if mem_info_list and isinstance(mem_info_list, list):
            mem = mem_info_list[-1] if mem_info_list else {}
            if isinstance(mem, dict):
                metrics.smem_used_bytes = int(mem.get("smem_used", 0) or 0)
                metrics.smem_total_bytes = int(mem.get("smem_total", 0) or 0)
                metrics.vram_used_bytes = int(mem.get("vram_used", 0) or 0)
                metrics.vram_total_bytes = int(mem.get("vram_total", 0) or 0)

    def _populate_frequency(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate frequency metrics."""
        dev_stats = dev.get("dev_stats", {})
        freqs_list = dev_stats.get("freqs", [])
        freq_limits = dev.get("freq_limits", [])
        
        if freqs_list and isinstance(freqs_list, list) and len(freqs_list) > 0:
            latest_freqs = freqs_list[-1] if freqs_list else []
            if latest_freqs and isinstance(latest_freqs, list) and len(latest_freqs) > 0:
                gt0_freq = latest_freqs[0] if latest_freqs else {}
                if isinstance(gt0_freq, dict):
                    metrics.freq_actual = float(gt0_freq.get("act_freq", 0) or 0)
                    metrics.freq_max = float(gt0_freq.get("max_freq", 0) or 0)

        if freq_limits and isinstance(freq_limits, list) and len(freq_limits) > 0:
            gt0_limits = freq_limits[0] if freq_limits else {}
            if isinstance(gt0_limits, dict) and metrics.freq_max == 0:
                metrics.freq_max = float(gt0_limits.get("maximum", 0) or 0)

    def _populate_power(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate power metrics."""
        dev_stats = dev.get("dev_stats", {})
        power_list = dev_stats.get("power", [])
        
        if power_list and isinstance(power_list, list) and len(power_list) > 0:
            power = power_list[-1] if power_list else {}
            if isinstance(power, dict):
                metrics.power_gpu = float(power.get("gpu_cur_power", 0) or 0)
                metrics.power_package = float(power.get("pkg_cur_power", 0) or 0)

    def _populate_temperature(self, metrics: GPUMetrics, dev: dict[str, Any]) -> None:
        """Populate temperature metrics."""
        dev_stats = dev.get("dev_stats", {})
        temps_list = dev_stats.get("temps", [])
        
        if temps_list and isinstance(temps_list, list) and len(temps_list) > 0:
            temp = temps_list[-1] if temps_list else 0
            if isinstance(temp, (int, float)):
                metrics.temperature = float(temp)
            elif isinstance(temp, dict):
                metrics.temperature = float(temp.get("value", temp.get("current", 0)) or 0)


def get_gpu_stats_dict(collector: Optional[QmassaCollector]) -> dict[str, Any]:
    """Get GPU stats as a dictionary suitable for API response.
    
    Args:
        collector: Optional QmassaCollector instance.
        
    Returns:
        Dictionary with GPU stats or error message.
    """
    if not collector:
        return {"available": False, "error": "GPU monitoring disabled"}

    metrics = collector.get_metrics()
    return metrics.to_dict()
