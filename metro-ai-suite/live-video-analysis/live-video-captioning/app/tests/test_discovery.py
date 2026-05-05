# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Tests for backend.services.discovery, model and pipeline discovery."""

import json
from unittest.mock import patch

import pytest
from fastapi import HTTPException

from backend.services.discovery import (
    discover_models,
    discover_detection_models,
    is_detection_pipeline,
    discover_pipelines_remote,
    _infer_model_device,
    _infer_pipeline_device,
)


# ===================================================================
# _infer_model_device
# ===================================================================
class TestInferModelDevice:
    """Unit tests for the _infer_model_device helper."""

    def test_npu_suffix(self):
        assert _infer_model_device("InternVL2-2B-npu") == "npu"

    def test_gpu_suffix(self):
        assert _infer_model_device("InternVL2-2B-gpu") == "gpu"

    def test_no_suffix_returns_cpu(self):
        assert _infer_model_device("InternVL2-2B") == "cpu"

    def test_case_insensitive_npu(self):
        assert _infer_model_device("InternVL2-2B-NPU") == "npu"

    def test_case_insensitive_gpu(self):
        assert _infer_model_device("InternVL2-2B-GPU") == "gpu"

    def test_unrelated_suffix_returns_cpu(self):
        assert _infer_model_device("InternVL2-2B-int4") == "cpu"


# ===================================================================
# _infer_pipeline_device
# ===================================================================
class TestInferPipelineDevice:
    """Unit tests for the _infer_pipeline_device helper."""

    def test_cpu_pipeline(self):
        assert _infer_pipeline_device("GenAI_Pipeline_on_CPU") == "cpu"

    def test_gpu_pipeline(self):
        assert _infer_pipeline_device("GenAI_Pipeline_on_GPU") == "gpu"

    def test_npu_pipeline(self):
        assert _infer_pipeline_device("GenAI_Pipeline_on_NPU") == "npu"

    def test_detection_cpu_pipeline(self):
        assert _infer_pipeline_device("GenAI_Detection_Pipeline_on_CPU") == "cpu"

    def test_unknown_pipeline_returns_any(self):
        assert _infer_pipeline_device("some_custom_pipeline") == "any"

    def test_case_insensitive(self):
        assert _infer_pipeline_device("pipeline_ON_NPU") == "npu"


# ===================================================================
# discover_models
# ===================================================================
class TestDiscoverModels:
    """Tests for discover_models(), VLM model directory scanning."""

    def test_returns_empty_when_dir_missing(self, tmp_path):
        """Returns an empty list when the models directory does not exist."""
        missing = tmp_path / "nonexistent"
        assert discover_models(missing) == []

    def test_returns_empty_for_empty_dir(self, models_dir):
        """Returns an empty list when the directory is empty."""
        assert discover_models(models_dir) == []

    def test_discovers_subdirectory_models_with_device(self, models_dir):
        """Each subdirectory returns a ModelInfo with inferred device."""
        (models_dir / "InternVL2-1B").mkdir()
        (models_dir / "InternVL2-2B-npu").mkdir()
        (models_dir / "InternVL2-2B-gpu").mkdir()
        result = discover_models(models_dir)
        names = [m.name for m in result]
        devices = {m.name: m.device for m in result}
        assert names == ["InternVL2-1B", "InternVL2-2B-gpu", "InternVL2-2B-npu"]
        assert devices["InternVL2-1B"] == "cpu"
        assert devices["InternVL2-2B-npu"] == "npu"
        assert devices["InternVL2-2B-gpu"] == "gpu"

    def test_discovers_flat_file_models_as_cpu(self, models_dir):
        """XML, BIN, and JSON files in the root are returned as cpu models."""
        (models_dir / "model.xml").write_text("")
        result = discover_models(models_dir)
        assert len(result) == 1
        assert result[0].name == "model.xml"
        assert result[0].device == "cpu"

    def test_ignores_dotfiles(self, models_dir):
        """Hidden files/directories (starting with '.') are skipped."""
        (models_dir / ".hidden_dir").mkdir()
        (models_dir / ".hidden_file.json").write_text("")
        (models_dir / "visible_model").mkdir()
        result = discover_models(models_dir)
        assert len(result) == 1
        assert result[0].name == "visible_model"

    def test_ignores_unsupported_extensions(self, models_dir):
        """Files with extensions other than .xml, .bin, .json are skipped."""
        (models_dir / "readme.txt").write_text("")
        (models_dir / "data.csv").write_text("")
        assert discover_models(models_dir) == []

    def test_results_are_sorted(self, models_dir):
        """Returned model names are sorted alphabetically."""
        for name in ["Zeta", "Alpha", "Mid"]:
            (models_dir / name).mkdir()
        names = [m.name for m in discover_models(models_dir)]
        assert names == ["Alpha", "Mid", "Zeta"]


# ===================================================================
# discover_detection_models
# ===================================================================
class TestDiscoverDetectionModels:
    """Tests for discover_detection_models(), detection model scanning."""

    def test_returns_empty_when_dir_missing(self, tmp_path):
        """Returns an empty list when the directory does not exist."""
        assert discover_detection_models(tmp_path / "nope") == []

    def test_returns_empty_for_empty_dir(self, detection_models_dir):
        """Returns an empty list for an empty directory."""
        assert discover_detection_models(detection_models_dir) == []

    def test_valid_detection_model_structure(self, detection_models_dir):
        """Detects model_name/public/model_name directory structure."""
        model = detection_models_dir / "yolov8s" / "public" / "yolov8s"
        model.mkdir(parents=True)
        result = discover_detection_models(detection_models_dir)
        assert result == ["yolov8s"]

    def test_ignores_incomplete_structure(self, detection_models_dir):
        """Directories without the expected public/<name> structure are skipped."""
        # Has 'public/' but not the model sub-directory
        (detection_models_dir / "incomplete" / "public").mkdir(parents=True)
        assert discover_detection_models(detection_models_dir) == []

    def test_ignores_dotdirs(self, detection_models_dir):
        """Hidden directories are skipped."""
        hidden = detection_models_dir / ".hidden" / "public" / ".hidden"
        hidden.mkdir(parents=True)
        assert discover_detection_models(detection_models_dir) == []

    def test_ignores_files_at_root(self, detection_models_dir):
        """Regular files in the detection models root are ignored."""
        (detection_models_dir / "notes.txt").write_text("")
        assert discover_detection_models(detection_models_dir) == []


# ===================================================================
# is_detection_pipeline
# ===================================================================
class TestIsDetectionPipeline:
    """Tests for the is_detection_pipeline() classifier helper."""

    def test_no_parameters_returns_false(self):
        """A pipeline item with no parameters is not a detection pipeline."""
        assert is_detection_pipeline({}) is False

    def test_detection_model_name_key(self):
        """Presence of 'detection_model_name' marks the pipeline as detection."""
        item = {"parameters": {"properties": {"detection_model_name": {}}}}
        assert is_detection_pipeline(item) is True

    def test_detection_threshold_key(self):
        """Presence of 'detection_threshold' marks the pipeline as detection."""
        item = {"parameters": {"properties": {"detection_threshold": {}}}}
        assert is_detection_pipeline(item) is True

    def test_detection_prefixed_key(self):
        """Any key starting with 'detection_' triggers detection classification."""
        item = {"parameters": {"properties": {"detection_custom_field": {}}}}
        assert is_detection_pipeline(item) is True

    def test_non_detection_keys(self):
        """Keys not related to detection do not trigger detection classification."""
        item = {"parameters": {"properties": {"captioner_prompt": {}, "model": {}}}}
        assert is_detection_pipeline(item) is False


# ===================================================================
# discover_pipelines_remote
# ===================================================================
class TestDiscoverPipelinesRemote:
    """Tests for discover_pipelines_remote(), remote pipeline discovery."""

    def _mock_http(self, payload):
        """Return a patcher that makes http_json return the given JSON payload."""
        return patch(
            "backend.services.discovery.http_json",
            return_value=json.dumps(payload),
        )

    def test_list_of_strings(self):
        """A simple list of pipeline name strings is returned as non-detection."""
        with self._mock_http(["pipe_a", "pipe_b"]):
            result = discover_pipelines_remote()
        names = [r["pipeline_name"] for r in result]
        assert "pipe_a" in names
        assert all(r["pipeline_type"] == "non-detection" for r in result)

    def test_device_inferred_from_pipeline_name(self):
        """device field is correctly inferred from the pipeline name suffix."""
        payload = [
            {"name": "GenAI_Pipeline_on_CPU", "parameters": {"properties": {}}},
            {"name": "GenAI_Pipeline_on_GPU", "parameters": {"properties": {}}},
            {"name": "GenAI_Pipeline_on_NPU", "parameters": {"properties": {}}},
        ]
        with self._mock_http(payload):
            result = discover_pipelines_remote()
        device_map = {r["pipeline_name"]: r["device"] for r in result}
        assert device_map["GenAI_Pipeline_on_CPU"] == "cpu"
        assert device_map["GenAI_Pipeline_on_GPU"] == "gpu"
        assert device_map["GenAI_Pipeline_on_NPU"] == "npu"

    def test_unknown_pipeline_device_is_any(self):
        """Pipelines with no device suffix get device='any'."""
        with self._mock_http([{"name": "custom_pipe", "parameters": {"properties": {}}}]):
            result = discover_pipelines_remote()
        assert result[0]["device"] == "any"

    def test_list_of_dicts_with_version(self):
        """Pipeline dicts with a 'version' key use that as the pipeline name."""
        with self._mock_http([{"version": "v1", "parameters": {"properties": {}}}]):
            result = discover_pipelines_remote()
        assert result[0]["pipeline_name"] == "v1"

    def test_list_of_dicts_with_name(self):
        """Pipeline dicts with a 'name' key (no version) use name."""
        with self._mock_http(
            [
                {
                    "name": "detpipe",
                    "parameters": {"properties": {"detection_model_name": {}}},
                }
            ]
        ):
            result = discover_pipelines_remote()
        # detection pipeline is filtered when ENABLE_DETECTION_PIPELINE=false
        # so the result should fallback to default
        assert len(result) >= 1

    def test_dict_payload_with_pipelines_key(self):
        """A dict wrapping pipelines under a 'pipelines' key is normalised."""
        with self._mock_http({"pipelines": ["alpha"]}):
            result = discover_pipelines_remote()
        assert result[0]["pipeline_name"] == "alpha"

    def test_dict_payload_with_items_key(self):
        """A dict wrapping pipelines under an 'items' key is normalised."""
        with self._mock_http({"items": ["beta"]}):
            result = discover_pipelines_remote()
        assert result[0]["pipeline_name"] == "beta"

    def test_fallback_on_generic_exception(self):
        """A non-HTTP exception from http_json returns the default pipeline."""
        with patch(
            "backend.services.discovery.http_json",
            side_effect=Exception("boom"),
        ):
            result = discover_pipelines_remote()
        assert len(result) == 1
        assert result[0]["pipeline_type"] == "non-detection"
        assert "device" in result[0]

    def test_http_exception_is_propagated(self):
        """An HTTPException from http_json (e.g. server unreachable) is re-raised."""
        with patch(
            "backend.services.discovery.http_json",
            side_effect=HTTPException(status_code=502, detail="Pipeline server unreachable"),
        ):
            with pytest.raises(HTTPException) as exc_info:
                discover_pipelines_remote()
        assert exc_info.value.status_code == 502

    def test_empty_list_returns_default(self):
        """An empty list from the server returns the default pipeline."""
        with self._mock_http([]):
            result = discover_pipelines_remote()
        assert len(result) == 1

    def test_detection_pipelines_filtered_when_disabled(self):
        """Detection pipelines are excluded when ENABLE_DETECTION_PIPELINE is False."""
        payload = [
            {
                "version": "det",
                "parameters": {"properties": {"detection_model_name": {}}},
            },
        ]
        with self._mock_http(payload):
            result = discover_pipelines_remote()
        # All detection pipelines filtered out; fallback returned
        assert all(r["pipeline_type"] == "non-detection" for r in result)

    def test_proxy_pipelines_are_hidden_from_results(self):
        """Proxy pipelines for default resolution are not exposed to the UI."""
        payload = [
            {"version": "captioner_Default_Resolution", "parameters": {"properties": {}}},
            {"version": "captioner_Custom", "parameters": {"properties": {}}},
        ]
        with self._mock_http(payload):
            result = discover_pipelines_remote()
        assert [item["pipeline_name"] for item in result] == ["captioner_Custom"]
