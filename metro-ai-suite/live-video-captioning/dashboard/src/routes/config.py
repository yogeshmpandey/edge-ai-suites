"""
Configuration route handler for the Live Video Captioning Dashboard.

Provides runtime configuration endpoint for the frontend client.
"""

import json

from aiohttp import web

from src.config import SIGNALING_URL, PEER_ID, METADATA_FILE


async def runtime_config(request: web.Request) -> web.Response:
    """
    ---
    summary: Get runtime configuration
    description: Returns JavaScript configuration for the frontend client.
    tags:
      - Configuration
    responses:
      "200":
        description: JavaScript runtime configuration
        content:
          application/javascript:
            schema:
              type: string
    """
    payload = {
        "signalingUrl": SIGNALING_URL,
        "defaultPeerId": PEER_ID,
        "defaultMetadataFile": METADATA_FILE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return web.Response(text=body, content_type="application/javascript")


# Route definitions for registration with Swagger
config_routes = [
    web.get("/runtime-config.js", runtime_config, allow_head=False),
]
