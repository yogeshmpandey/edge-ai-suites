# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""
Health check routes for the Metrics Service.
"""

import logging

from fastapi import APIRouter

from .relay import get_status
from .poller import get_poller

router = APIRouter(tags=["health"])
logger = logging.getLogger("live-metrics-service.health")


@router.get("/health")
async def health_check():
    """
    Basic health check endpoint.
    
    Returns:
        {"status": "healthy"}
    """
    return {"status": "healthy"}


@router.get("/api/health")
async def api_health_check():
    """
    Detailed health check with service status.
    
    Returns:
        {
            "status": "healthy",
            "collector_connected": bool,
            "clients_connected": int,
            "poller_active": bool,
            "poller_target": str
        }
    """
    relay_status = await get_status()
    poller = get_poller()
    
    return {
        "status": "healthy",
        "collector_connected": relay_status["collector_connected"],
        "clients_connected": relay_status["clients_connected"],
        "poller_active": poller.is_running,
        "poller_target": poller.full_url if poller.is_configured else None,
    }


@router.get("/api/metrics/status")
async def metrics_status():
    """
    Get the current status of metrics collection.

    Returns:
        {
            "collector_connected": bool,
            "clients_connected": int,
            "poller": {
                "active": bool,
                "target": str,
                "last_metrics": dict or null
            }
        }
    """
    relay_status = await get_status()
    poller = get_poller()

    return {
        "collector_connected": relay_status["collector_connected"],
        "clients_connected": relay_status["clients_connected"],
        "poller": {
            "active": poller.is_running,
            "target": poller.full_url if poller.is_configured else None,
            "last_metrics": poller.last_metrics,
        },
    }
