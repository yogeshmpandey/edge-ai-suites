"""
Pipelines route handler for the Live Video Captioning Dashboard.

Provides endpoint for listing available video processing pipelines.
"""

from aiohttp import web

from src.services.discovery import discover_pipelines_remote


async def list_pipelines(request: web.Request) -> web.Response:
    """
    ---
    summary: List available pipelines
    description: Returns a list of available video processing pipelines from the pipeline server.
    tags:
      - Pipelines
    responses:
      "200":
        description: List of available pipelines
        content:
          application/json:
            schema:
              type: object
              properties:
                pipelines:
                  type: array
                  items:
                    type: string
                  description: List of pipeline names
    """
    session = request.app["http_session"]
    names = await discover_pipelines_remote(session)
    return web.json_response({"pipelines": names})


# Route definitions for registration with Swagger
pipelines_routes = [
    web.get("/api/pipelines", list_pipelines, allow_head=False),
]
