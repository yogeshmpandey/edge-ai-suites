"""
GPU metrics data model for the Live Video Captioning Dashboard.

Contains the GPUMetrics dataclass for GPU telemetry data.
"""

from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class GPUMetrics:
    """GPU metrics collected from qmassa.
    
    Attributes:
        available: Whether GPU monitoring is available.
        device_name: GPU device name.
        driver: Driver name (xe, i915, amdgpu).
        pci_slot: PCI slot identifier.
        device_type: "Integrated", "Discrete", or "Unknown".
        engines: Engine utilization by name (0-100%).
        total_engine_percent: Average engine utilization.
        smem_used_bytes: System memory used by GPU.
        smem_total_bytes: Total system memory allocated to GPU.
        vram_used_bytes: Device VRAM used (discrete GPUs).
        vram_total_bytes: Total VRAM (discrete GPUs).
        freq_actual: Current GPU frequency (MHz).
        freq_max: Maximum GPU frequency (MHz).
        power_gpu: GPU power consumption (Watts).
        power_package: Package power consumption (Watts).
        temperature: GPU temperature (Celsius).
        error: Error message if monitoring failed.
    """
    available: bool = False
    device_name: str = ""
    driver: str = ""
    pci_slot: str = ""
    device_type: str = ""
    # Engine utilization (0-100 percent)
    engines: dict[str, float] = field(default_factory=dict)
    total_engine_percent: float = 0.0
    # Memory stats
    smem_used_bytes: int = 0
    smem_total_bytes: int = 0
    vram_used_bytes: int = 0
    vram_total_bytes: int = 0
    # Frequency (MHz)
    freq_actual: float = 0.0
    freq_max: float = 0.0
    # Power (Watts)
    power_gpu: float = 0.0
    power_package: float = 0.0
    # Temperature (Celsius)
    temperature: float = 0.0
    # Error message if any
    error: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization.
        
        Returns formatted GPU stats suitable for API response.
        """
        if self.error:
            return {"available": False, "error": self.error}

        if not self.available:
            return {"available": False, "error": "No GPU data available"}

        result: dict[str, Any] = {
            "available": True,
            "device_name": self.device_name,
            "driver": self.driver,
            "device_type": self.device_type,
            "usage_percent": round(self.total_engine_percent, 1),
            "engines": {k: round(v, 1) for k, v in self.engines.items()},
        }

        # VRAM (device memory - for discrete GPUs)
        if self.vram_total_bytes > 0:
            result["vram_used_gb"] = round(self.vram_used_bytes / (1024**3), 2)
            result["vram_total_gb"] = round(self.vram_total_bytes / (1024**3), 2)
            result["vram_percent"] = round(
                (self.vram_used_bytes / self.vram_total_bytes) * 100, 1
            ) if self.vram_total_bytes else None

        # System memory used by GPU
        if self.smem_total_bytes > 0:
            result["smem_used_gb"] = round(self.smem_used_bytes / (1024**3), 2)
            result["smem_total_gb"] = round(self.smem_total_bytes / (1024**3), 2)

        # Frequency
        if self.freq_actual > 0:
            result["freq_actual"] = round(self.freq_actual)
        if self.freq_max > 0:
            result["freq_max"] = round(self.freq_max)

        # Power
        if self.power_gpu > 0:
            result["power_gpu"] = round(self.power_gpu, 1)
        if self.power_package > 0:
            result["power_package"] = round(self.power_package, 1)

        # Temperature
        if self.temperature > 0:
            result["temperature"] = round(self.temperature, 1)

        return result
