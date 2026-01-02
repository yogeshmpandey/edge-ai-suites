"""
Models route handler for the Live Video Captioning Dashboard.

Provides endpoint for listing available captioning models.
"""

from aiohttp import web

from src.config import MODELS_DIR
from src.services.discovery import discover_models


async def list_models(request: web.Request) -> web.Response:
    """
    ---
    summary: List available models
    description: Returns a list of available captioning models from the models directory.
    tags:
      - Models
    responses:
      "200":
        description: List of available models
        content:
          application/json:
            schema:
              type: object
              properties:
                models:
                  type: array
                  items:
                    type: string
                  description: List of model names
    """
    models = discover_models(MODELS_DIR)
    return web.json_response({"models": models})


# Route definitions for registration with Swagger
models_routes = [
    web.get("/api/models", list_models, allow_head=False),
]
