import json

from fastapi import APIRouter, Response

from ..config import AGENT_MODE, METADATA_FILE, PEER_ID, SIGNALING_URL

router = APIRouter()


@router.get("/runtime-config.js")
async def runtime_config() -> Response:
    """Return runtime configuration as JavaScript for the frontend."""
    payload = {
        "signalingUrl": SIGNALING_URL,
        "defaultPeerId": PEER_ID,
        "defaultMetadataFile": METADATA_FILE,
        "agentMode": AGENT_MODE,
    }
    body = f"window.RUNTIME_CONFIG = {json.dumps(payload)};"
    return Response(content=body, media_type="application/javascript")
