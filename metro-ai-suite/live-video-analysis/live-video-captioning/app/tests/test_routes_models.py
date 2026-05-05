# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Tests for backend.routes.models, VLM and detection model listing endpoints."""

from unittest.mock import patch

from backend.models.responses import ModelInfo


class TestListVlmModels:
    """GET /api/vlm-models endpoint."""

    def test_returns_empty_list_when_no_models(self, client):
        """Returns an empty model list when the models directory is empty."""
        with patch("backend.routes.models.discover_models", return_value=[]):
            resp = client.get("/api/vlm-models")
        assert resp.status_code == 200
        assert resp.json() == {"models": []}

    def test_returns_discovered_models_with_device(self, client):
        """Returns ModelInfo objects with name and device fields."""
        models = [
            ModelInfo(name="InternVL2-1B", device="cpu"),
            ModelInfo(name="InternVL2-2B-gpu", device="gpu"),
            ModelInfo(name="InternVL2-2B-npu", device="npu"),
        ]
        with patch("backend.routes.models.discover_models", return_value=models):
            resp = client.get("/api/vlm-models")
        assert resp.status_code == 200
        assert resp.json()["models"] == [
            {"name": "InternVL2-1B", "device": "cpu"},
            {"name": "InternVL2-2B-gpu", "device": "gpu"},
            {"name": "InternVL2-2B-npu", "device": "npu"},
        ]

    def test_cpu_model_has_no_device_suffix(self, client):
        """A model with no suffix is tagged as cpu."""
        models = [ModelInfo(name="InternVL2-2B", device="cpu")]
        with patch("backend.routes.models.discover_models", return_value=models):
            resp = client.get("/api/vlm-models")
        assert resp.status_code == 200
        assert resp.json()["models"][0]["device"] == "cpu"


class TestListDetectionModels:
    """GET /api/detection-models endpoint."""

    def test_returns_empty_list_when_no_models(self, client):
        """Returns an empty list when no detection models exist."""
        with patch("backend.routes.models.discover_detection_models", return_value=[]):
            resp = client.get("/api/detection-models")
        assert resp.status_code == 200
        assert resp.json() == {"models": []}

    def test_returns_discovered_detection_models(self, client):
        """Returns detection model names from the directory."""
        models = ["yolov8s"]
        with patch(
            "backend.routes.models.discover_detection_models", return_value=models
        ):
            resp = client.get("/api/detection-models")
        assert resp.status_code == 200
        assert resp.json()["models"] == models
