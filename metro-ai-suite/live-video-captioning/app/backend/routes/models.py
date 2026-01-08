from fastapi import APIRouter

from ..config import MODELS_DIR
from ..models import ModelList
from ..services import discover_models

router = APIRouter(prefix="/api", tags=["models"])


@router.get("/models", response_model=ModelList)
async def list_models() -> ModelList:
    """List available VLM models from the models directory."""
    models = discover_models(MODELS_DIR)
    return ModelList(models=models)
