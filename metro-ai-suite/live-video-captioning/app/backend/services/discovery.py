import json
from pathlib import Path

from ..config import PIPELINE_NAME, PIPELINE_SERVER_URL
from .http_client import http_json


def discover_models(root: Path) -> list[str]:
    """Discover available models from the models directory."""
    if not root.exists():
        return []
    models: list[str] = []
    for entry in sorted(root.iterdir()):
        if entry.name.startswith("."):
            continue
        if entry.is_dir():
            models.append(entry.name)
        else:
            # Allow flat exports placed directly under ov_models
            if entry.suffix in {".xml", ".bin", ".json"}:
                models.append(entry.name)
    return models


def discover_pipelines_remote() -> list[str]:
    """Discover available pipelines from the pipeline server."""
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines"
    try:
        raw = http_json("GET", url)
        payload = json.loads(raw)
        # Accept either list[str] or list[dict {name}] or {'pipelines': [...]}
        if isinstance(payload, list):
            names = []
            for item in payload:
                if isinstance(item, str):
                    names.append(item)
                elif isinstance(item, dict) and isinstance(item.get("version"), str):
                    names.append(item["version"])
            return names or [PIPELINE_NAME]
        if isinstance(payload, dict):
            items = payload.get("pipelines") or payload.get("items") or []
            if isinstance(items, list):
                names = []
                for item in items:
                    if isinstance(item, str):
                        names.append(item)
                    elif isinstance(item, dict) and isinstance(
                        item.get("version"), str
                    ):
                        names.append(item["version"])
                return names or [PIPELINE_NAME]
    except Exception:
        return [PIPELINE_NAME]
    return [PIPELINE_NAME]
