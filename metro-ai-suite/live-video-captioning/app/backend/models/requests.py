from typing import Optional

from pydantic import BaseModel, Field


class StartRunRequest(BaseModel):
    rtspUrl: str = Field(..., min_length=1)
    prompt: str = Field(default="Describe what you see in the image in one sentence.")
    detectionModelName: Optional[str] = Field(default="yolov8s")
    detectionThreshold: Optional[float] = Field(default=0.5, ge=0.0, le=1.0)
    modelName: str = Field(default="OpenGVLab/InternVL2-2B")
    maxNewTokens: int = Field(default=70, ge=1, le=4096)
    pipelineName: Optional[str] = Field(default=None)
    runName: Optional[str] = Field(default=None)
