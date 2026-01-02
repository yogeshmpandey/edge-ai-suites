"""
Monitoring route handlers for the Live Video Captioning Dashboard.

Provides endpoint for GPU statistics.
"""

from aiohttp import web

from src.services.gpu_collector import QmassaCollector, get_gpu_stats_dict


async def gpu_stats(request: web.Request) -> web.Response:
    """
    ---
    summary: Get current GPU stats
    description: Returns current GPU statistics as a JSON response.
    tags:
      - Monitoring
    responses:
      "200":
        description: Current GPU statistics
        content:
          application/json:
            schema:
              type: object
              properties:
                available:
                  type: boolean
                  description: Whether GPU monitoring is available
                device_name:
                  type: string
                  description: GPU device name
                driver:
                  type: string
                  description: GPU driver name
                device_type:
                  type: string
                  description: "Device type (Integrated, Discrete, Unknown)"
                usage_percent:
                  type: number
                  description: GPU usage percentage
                engines:
                  type: object
                  description: Engine utilization breakdown
                vram_used_gb:
                  type: number
                  description: VRAM used in GB
                vram_total_gb:
                  type: number
                  description: Total VRAM in GB
                vram_percent:
                  type: number
                  description: VRAM usage percentage
                freq_actual:
                  type: number
                  description: Current GPU frequency in MHz
                freq_max:
                  type: number
                  description: Maximum GPU frequency in MHz
                power_gpu:
                  type: number
                  description: GPU power consumption in Watts
                temperature:
                  type: number
                  description: GPU temperature in Celsius
                error:
                  type: string
                  description: Error message if monitoring failed
    """
    collector: QmassaCollector | None = request.app.get("gpu_collector")
    return web.json_response(get_gpu_stats_dict(collector))


# Route definitions for registration with Swagger
monitoring_routes = [
    web.get("/api/gpu-stats", gpu_stats, allow_head=False),
]
