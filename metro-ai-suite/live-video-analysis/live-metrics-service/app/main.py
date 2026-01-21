# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""
Metrics Service - A reusable microservice for system metrics collection and relay.

This service provides:
1. WebSocket relay for metrics from collectors (Telegraf) to clients
2. Optional polling of target services for metrics collection
3. REST API endpoints for health and status checks

The service is designed to be deployed alongside any application that needs
system metrics visualization, not just for live-video-captioning.
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import METRICS_PORT, LOG_LEVEL, CORS_ORIGINS
from .relay import router as relay_router
from .routes import router as health_router
from .poller import start_poller, stop_poller

# Configure logging
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL.upper(), logging.INFO),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("live-metrics-service")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle - startup and shutdown."""
    logger.info("Starting Metrics Service...")
    
    # Start the poller if configured
    try:
        await start_poller()
    except Exception as e:
        logger.warning(f"Failed to start poller: {e}")
    
    yield
    
    # Shutdown: Clean up poller
    logger.info("Shutting down Metrics Service...")
    await stop_poller()


app = FastAPI(
    title="Metrics Service",
    description="A reusable microservice for system metrics collection and relay",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(relay_router)
app.include_router(health_router)


@app.get("/")
async def root():
    """Root endpoint with service information."""
    return {
        "service": "Metrics Service",
        "version": "1.0.0",
        "description": "A reusable microservice for system metrics collection and relay",
        "endpoints": {
            "websocket_collector": "/ws/collector",
            "websocket_clients": "/ws/clients",
            "health": "/health",
            "health_detailed": "/api/health",
            "metrics_status": "/api/metrics/status",
        },
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("app.main:app", host="0.0.0.0", port=METRICS_PORT, reload=True)
