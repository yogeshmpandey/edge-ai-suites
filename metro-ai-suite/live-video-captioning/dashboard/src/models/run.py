"""
Run data models for the Live Video Captioning Dashboard.

Contains dataclasses for run requests and run state.
"""

from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class StartRunRequest:
    """Request payload for starting a new video processing run.
    
    Attributes:
        rtspUrl: The RTSP URL of the video source (required).
        prompt: The prompt to use for video captioning.
        modelName: The name of the model to use for captioning.
        maxNewTokens: Maximum number of new tokens to generate (1-4096).
        pipelineName: Optional pipeline name override.
    """
    rtspUrl: str
    prompt: str = "Describe what you see in the image in one sentence."
    modelName: str = "OpenGVLab/InternVL2-2B"
    maxNewTokens: int = 70
    pipelineName: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StartRunRequest":
        """Create a StartRunRequest from a dictionary.
        
        Args:
            data: Dictionary containing request parameters.
            
        Returns:
            StartRunRequest instance.
            
        Raises:
            ValueError: If required fields are missing or invalid.
        """
        if "rtspUrl" not in data or not data["rtspUrl"]:
            raise ValueError("rtspUrl is required and must not be empty")
        
        max_tokens = data.get("maxNewTokens", 70)
        if not isinstance(max_tokens, int) or max_tokens < 1 or max_tokens > 4096:
            raise ValueError("maxNewTokens must be an integer between 1 and 4096")
        
        return cls(
            rtspUrl=data["rtspUrl"],
            prompt=data.get("prompt", cls.prompt),
            modelName=data.get("modelName", cls.modelName),
            maxNewTokens=max_tokens,
            pipelineName=data.get("pipelineName"),
        )


@dataclass
class RunInfo:
    """Information about an active video processing run.
    
    Attributes:
        runId: Unique identifier for this run.
        pipelineId: Pipeline server's identifier for the pipeline instance.
        peerId: WebRTC peer ID for video streaming.
        metadataFile: Path to the metadata output file.
        modelName: Name of the model used for captioning.
        pipelineName: Name of the pipeline configuration used.
    """
    runId: str
    pipelineId: str
    peerId: str
    metadataFile: str
    modelName: str = ""
    pipelineName: str = ""

    def to_dict(self) -> dict[str, str]:
        """Convert to dictionary for JSON serialization."""
        return {
            "runId": self.runId,
            "pipelineId": self.pipelineId,
            "peerId": self.peerId,
            "metadataFile": self.metadataFile,
            "modelName": self.modelName,
            "pipelineName": self.pipelineName,
        }
