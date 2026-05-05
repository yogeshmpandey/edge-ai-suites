import json
from pathlib import Path
from typing import List, Dict
from fastapi import HTTPException
from ..config import PIPELINE_NAME, PIPELINE_SERVER_URL, ENABLE_DETECTION_PIPELINE
from ..models import ModelInfo, PipelineInfo, ModelList, PipelineInfoList
from .http_client import http_json


def _infer_model_device(name: str) -> str:
    """Infer target device from the model directory name suffix convention.

    Naming convention written by download_models.sh:
      <model>-npu  → npu
      <model>-gpu  → gpu
      <model>      → cpu  (default, no suffix)
    """
    lower = name.lower()
    if lower.endswith("-npu"):
        return "npu"
    if lower.endswith("-gpu"):
        return "gpu"
    return "cpu"


def _infer_pipeline_device(name: str) -> str:
    """Infer target device from the pipeline name.

    DL Streamer pipeline naming convention:
      *_on_NPU  → npu
      *_on_GPU  → gpu
      *_on_CPU  → cpu
      anything else → any
    """
    upper = name.upper()
    if upper.endswith("_ON_NPU"):
        return "npu"
    if upper.endswith("_ON_GPU"):
        return "gpu"
    if upper.endswith("_ON_CPU"):
        return "cpu"
    return "any"


def discover_models(root: Path) -> List[ModelInfo]:
    """Discover available models from the models directory.

    Returns a list of ModelInfo objects with name and inferred device tag.
    """
    if not root.exists():
        return []
    models: List[ModelInfo] = []
    for entry in sorted(root.iterdir()):
        if entry.name.startswith("."):
            continue
        if entry.is_dir():
            models.append(ModelInfo(name=entry.name, device=_infer_model_device(entry.name)))
        else:
            # Allow flat exports placed directly under ov_models
            if entry.suffix in {".xml", ".bin", ".json"}:
                models.append(ModelInfo(name=entry.name, device="cpu"))
    return models


def discover_detection_models(root: Path) -> List[str]:
    """Discover available detection models from the detection models directory."""
    if not root.exists():
        return []
    models: List[str] = []
    for entry in sorted(root.iterdir()):
        if entry.name.startswith("."):
            continue
        if entry.is_dir():
            # Check if this directory has the expected structure: model_name/public/model_name
            public_dir = entry / "public"
            if public_dir.exists() and public_dir.is_dir():
                # Check if there's a subdirectory with the same name as the parent
                model_subdir = public_dir / entry.name
                if model_subdir.exists() and model_subdir.is_dir():
                    models.append(entry.name)
    return models


def is_detection_pipeline(item: dict) -> bool:
    """Check if the given pipeline item represents a detection pipeline."""
    props = item.get("parameters", {}).get("properties", {})
    if isinstance(props, dict):
        # Any explicit detection fields
        detection_keys = {
            "detection_model_name",
            "detection_threshold",
        }
        # Either keys exist, or any key startswith 'detection_'
        if any(k in props for k in detection_keys):
            return True
        if any(
            isinstance(k, str) and k.lower().startswith("detection_")
            for k in props.keys()
        ):
            return True

    return False


def discover_pipelines_remote() -> List[Dict[str, str]]:
    """
    Discover available pipelines from the pipeline server and return a List of dicts:
    {
      "pipeline_name": <name>,
      "pipeline_type": "detection" | "non-detection",
      "device": "cpu" | "gpu" | "npu" | "any"
    }

    Behavior:
    - Normalizes payload that may be List[str], List[dict], or dict with 'pipelines'/'items'
    - Classifies using is_detection_pipeline(item) when item is a dict
    - Defaults string-only items to 'non-detection' (no metadata to inspect)
    - Optionally filters out detection pipelines when ENABLE_DETECTION_PIPELINE is False
    - Infers device from pipeline name suffix (_on_CPU/GPU/NPU)
    """
    url = f"{PIPELINE_SERVER_URL.rstrip('/')}/pipelines"
    try:
        raw = http_json("GET", url)
        payload = json.loads(raw)

        # Normalize to a List of items
        if isinstance(payload, List):
            items = payload
        elif isinstance(payload, dict):
            items = payload.get("pipelines") or payload.get("items") or []
        else:
            items = []

        if not isinstance(items, List):
            # Fallback to a single default pipeline
            return [{
                "pipeline_name": PIPELINE_NAME,
                "pipeline_type": "non-detection",
                "device": _infer_pipeline_device(PIPELINE_NAME),
            }]

        results: List[Dict[str, str]] = []

        for item in items:
            # Determine pipeline name
            if isinstance(item, str):
                name = item
                pipeline_type = "non-detection"  # No metadata available
            elif isinstance(item, dict):
                # Preserve your original preference for 'version' as name
                if isinstance(item.get("version"), str):
                    name = item["version"]
                elif isinstance(item.get("name"), str):
                    name = item["name"]
                elif isinstance(item.get("id"), str):
                    name = item["id"]
                else:
                    # No usable identifier
                    continue

                pipeline_type = (
                    "detection" if is_detection_pipeline(item) else "non-detection"
                )
            else:
                continue

            results.append({
                "pipeline_name": name,
                "pipeline_type": pipeline_type,
                "device": _infer_pipeline_device(name),
            })

        # Optional filtering based on your existing flag
        if not ENABLE_DETECTION_PIPELINE:
            results = [r for r in results if r["pipeline_type"] != "detection"]

        # Filter out proxy pipelines (hidden from UI, used internally for default resolution)
        results = [r for r in results if not r["pipeline_name"].endswith("_Default_Resolution")]

        # Fallback if nothing usable left
        if not results:
            return [{
                "pipeline_name": PIPELINE_NAME,
                "pipeline_type": "non-detection",
                "device": _infer_pipeline_device(PIPELINE_NAME),
            }]

        return results

    except HTTPException:
        raise
    except Exception:
        # Conservative fallback for parse / unexpected errors
        return [{
            "pipeline_name": PIPELINE_NAME,
            "pipeline_type": "non-detection",
            "device": _infer_pipeline_device(PIPELINE_NAME),
        }]
