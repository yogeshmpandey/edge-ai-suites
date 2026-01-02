"""
Live Video Captioning Dashboard - aiohttp Server

This module provides an async HTTP server using aiohttp for the live video
captioning dashboard. It exposes REST APIs for managing video processing
pipelines, streaming metadata, and monitoring system/GPU metrics.

API Documentation is available at /api/docs (Swagger UI).

Architecture:
    This is the application entry point that wires together:
    - Configuration from src.config
    - Route handlers from src.routes.*
    - Services from src.services.*
    - Models from src.models.*
"""

import logging

from aiohttp import web
import aiohttp_cors
from aiohttp_swagger3 import SwaggerDocs, SwaggerInfo, SwaggerUiSettings

from src.config import (
    APP_PORT,
    PUBLIC_DIR,
    QMASSA_ENABLED,
    QMASSA_JSON_FILE,
    QMASSA_POLL_INTERVAL_MS,
)
from src.utils.http import create_http_session
from src.services.gpu_collector import QmassaCollector

# Import route definitions
from src.routes.config import config_routes
from src.routes.models import models_routes
from src.routes.pipelines import pipelines_routes
from src.routes.runs import runs_routes
from src.routes.streaming import streaming_routes
from src.routes.monitoring import monitoring_routes

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def index_handler(request: web.Request) -> web.FileResponse:
    """
    ---
    summary: Serve the main dashboard page
    description: Returns the main HTML page for the src.
    tags:
      - Static
    responses:
      "200":
        description: HTML page
        content:
          text/html:
            schema:
              type: string
    """
    return web.FileResponse(PUBLIC_DIR / "index.html")


async def on_startup(app: web.Application) -> None:
    """Initialize resources on application startup.
    
    - Initializes the runs dictionary for tracking active runs.
    - Creates the HTTP client session for pipeline server communication.
    - Starts the GPU metrics collector if enabled.
    """
    logger.info("Starting Live Video Captioning Dashboard...")
    
    # Initialize runs storage
    app["runs"] = {}
    
    # Create HTTP session for pipeline server
    app["http_session"] = await create_http_session()
    logger.info("HTTP session created")
    
    # Start GPU collector if enabled
    if QMASSA_ENABLED:
        collector = QmassaCollector(QMASSA_JSON_FILE, QMASSA_POLL_INTERVAL_MS)
        await collector.start()
        app["gpu_collector"] = collector
        logger.info("GPU metrics collector started")
    
    logger.info(f"Dashboard ready on port {APP_PORT}")


async def on_cleanup(app: web.Application) -> None:
    """Cleanup resources on application shutdown.
    
    - Stops the GPU metrics collector.
    - Closes the HTTP client session.
    """
    logger.info("Shutting down Live Video Captioning Dashboard...")
    
    # Stop GPU collector
    collector = app.get("gpu_collector")
    if collector:
        await collector.stop()
        logger.info("GPU metrics collector stopped")
    
    # Close HTTP session
    session = app.get("http_session")
    if session and not session.closed:
        await session.close()
        logger.info("HTTP session closed")
    
    logger.info("Dashboard shutdown complete")


def create_app() -> web.Application:
    """Create and configure the aiohttp web application.
    
    Returns:
        Configured aiohttp Application instance with all routes and middleware.
    """
    app = web.Application()
    
    # Setup Swagger documentation
    swagger = SwaggerDocs(
        app,
        swagger_ui_settings=SwaggerUiSettings(path="/api/docs"),
        info=SwaggerInfo(
            title="Live Video Captioning Dashboard API",
            version="1.0.0",
            description="""
REST API for the Live Video Captioning Dashboard.

## Features

- **Runs Management**: Create, list, and stop video processing runs
- **Models**: List available captioning models
- **Pipelines**: List available video processing pipelines
- **Monitoring**: Real-time CPU, memory, and GPU statistics
- **Streaming**: Server-Sent Events for metadata and stats

## Authentication

Currently no authentication is required.

## Rate Limiting

No rate limiting is applied.
            """,
        ),
    )
    
    # Setup CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*",
        )
    })
    
    # Register all API routes with Swagger documentation
    swagger.add_routes(config_routes)
    swagger.add_routes(models_routes)
    swagger.add_routes(pipelines_routes)
    swagger.add_routes(runs_routes)
    swagger.add_routes(streaming_routes)
    swagger.add_routes(monitoring_routes)
    
    # Add root handler and static files (outside swagger for proper static file handling)
    app.router.add_get("/", index_handler)
    app.router.add_static("/", PUBLIC_DIR, name="static", show_index=False)
    
    # Apply CORS to all routes
    for route in list(app.router.routes()):
        try:
            cors.add(route)
        except ValueError:
            # Some routes may not support CORS
            pass
    
    # Register lifecycle handlers
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    
    return app


# Create the application instance
app = create_app()


if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=APP_PORT)
