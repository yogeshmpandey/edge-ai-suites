# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Tests for the intersections configuration model and loader."""

import pytest
import yaml

from model.broker import Broker
from model.intersection import Camera, Intersection
from service import intersection_config

FULL_CONFIG = """
intersections:
  - id: si1
    name: Main Street and 1st Ave
    ip: 10.0.0.11
    cameras:
      - name: si1-camera1
      - name: si1-camera2
        url: rtsp://10.0.0.21:8555/camera2
      - name: si1-camera3
      - name: si1-camera4
        url: rtsp://10.0.0.11:8554/custom-path
  - name: Broadway and 5th
    ip: 10.0.0.12
    cameras:
      - name: si2-camera1
      - name: si2-camera2
      - name: si2-camera3
      - name: si2-camera4
"""


@pytest.fixture
def config_file(tmp_path):
    def _write(content):
        path = tmp_path / "intersections.yaml"
        path.write_text(content)
        return str(path)

    return _write


class TestIntersectionModel:
    def test_cameras_default_to_four_named_after_the_id(self):
        intersection = Intersection(id="si1", name="Main", ip="10.0.0.11")
        assert [c.name for c in intersection.cameras] == [
            "si1-camera1",
            "si1-camera2",
            "si1-camera3",
            "si1-camera4",
        ]
        assert all(c.url is None for c in intersection.cameras)

    def test_camera_url_is_independent_per_camera(self):
        intersection = Intersection(
            id="si1",
            ip="10.0.0.11",
            cameras=[
                Camera(name="si1-camera1"),
                Camera(name="si1-camera2", url="rtsp://10.0.0.99:8554/camera2"),
            ],
        )
        assert intersection.cameras[0].url is None
        assert intersection.cameras[1].url == "rtsp://10.0.0.99:8554/camera2"

    def test_id_derived_from_camera_prefix_and_name_defaults_to_id(self):
        intersection = Intersection(ip="10.0.0.12", cameras=[Camera(name="si2-camera1")])
        assert intersection.id == "si2"
        assert intersection.name == "si2"

    def test_id_cannot_be_derived_without_cameras(self):
        with pytest.raises(ValueError):
            Intersection(ip="10.0.0.13")

    def test_rtsp_url_defaults_and_explicit_url(self):
        camera = Camera(name="si1-camera3")
        assert camera.rtsp_url("10.0.0.11") == "rtsp://10.0.0.11:8554/camera3"
        explicit = Camera(name="si1-camera4", url="rtsp://10.0.0.21:8555/custom-path")
        assert explicit.rtsp_url("1.2.3.4") == "rtsp://10.0.0.21:8555/custom-path"

    def test_to_broker_maps_ip_to_host(self):
        broker = Intersection(id="si1", name="Main", ip="10.0.0.11", mqtt_port=8883).to_broker()
        assert (broker.id, broker.name, broker.host, broker.port) == ("si1", "Main", "10.0.0.11", 8883)
        assert broker.type == "scenescape"

    def test_from_broker_round_trip_keeps_cameras(self):
        original = Intersection(id="si1", name="Main", ip="10.0.0.11", use_tls=False, enabled=False)
        restored = Intersection.from_broker(original.to_broker(), original.cameras)
        assert restored.model_dump() == original.model_dump()


class TestLoadIntersections:
    def test_missing_file_returns_empty_list(self, tmp_path):
        assert intersection_config.load_intersections(str(tmp_path / "absent.yaml")) == []

    def test_empty_config_returns_empty_list(self, config_file):
        assert intersection_config.load_intersections(config_file("intersections: []\n")) == []

    def test_loads_entries_with_defaults_applied(self, config_file):
        intersections = intersection_config.load_intersections(config_file(FULL_CONFIG))
        assert [i.id for i in intersections] == ["si1", "si2"]

        si1 = intersections[0]
        assert si1.name == "Main Street and 1st Ave"
        assert si1.ip == "10.0.0.11"
        assert [c.rtsp_url(si1.ip) for c in si1.cameras] == [
            "rtsp://10.0.0.11:8554/camera1",
            "rtsp://10.0.0.21:8555/camera2",
            "rtsp://10.0.0.11:8554/camera3",
            "rtsp://10.0.0.11:8554/custom-path",
        ]

    def test_invalid_entry_is_skipped(self, config_file, caplog):
        path = config_file(
            "intersections:\n"
            "  - id: si1\n"
            "    name: no ip here\n"
            "  - id: si2\n"
            "    ip: 10.0.0.12\n"
        )
        intersections = intersection_config.load_intersections(path)
        assert [i.id for i in intersections] == ["si2"]

    def test_malformed_yaml_returns_empty_list(self, config_file):
        assert intersection_config.load_intersections(config_file("intersections: [\n  - broken")) == []


class TestSaveIntersections:
    def test_round_trip(self, config_file, tmp_path):
        path = config_file(FULL_CONFIG)
        loaded = intersection_config.load_intersections(path)

        out = str(tmp_path / "out.yaml")
        assert intersection_config.save_intersections(out, loaded) is True

        with open(out) as f:
            raw = yaml.safe_load(f)
        assert [e["id"] for e in raw["intersections"]] == ["si1", "si2"]
        assert intersection_config.load_intersections(out) == loaded

    def test_write_failure_is_reported(self, tmp_path):
        unwritable = str(tmp_path / "missing-dir" / "out.yaml")
        assert intersection_config.save_intersections(unwritable, []) is False


class TestBrokerConversion:
    def test_to_brokers(self, config_file):
        brokers = intersection_config.to_brokers(intersection_config.load_intersections(config_file(FULL_CONFIG)))
        assert [(b.id, b.host, b.port) for b in brokers] == [
            ("si1", "10.0.0.11", 1883),
            ("si2", "10.0.0.12", 1883),
        ]

    def test_merge_keeps_known_cameras(self, config_file):
        existing = intersection_config.load_intersections(config_file(FULL_CONFIG))
        brokers = [b.model_dump() for b in intersection_config.to_brokers(existing)]

        merged = intersection_config.merge_brokers(brokers, existing)
        assert [c.name for c in merged[0].cameras] == [c.name for c in existing[0].cameras]
        assert merged[0].cameras[1].url == "rtsp://10.0.0.21:8555/camera2"

    def test_merge_keeps_camera_urls_unchanged_when_host_changes(self, config_file):
        existing = intersection_config.load_intersections(config_file(FULL_CONFIG))
        brokers = [b.model_dump() for b in intersection_config.to_brokers(existing)]
        brokers[0]["host"] = "10.0.0.77"

        merged = intersection_config.merge_brokers(brokers, existing)
        assert merged[0].ip == "10.0.0.77"
        # camera URLs are self-contained and independent of the broker host
        assert merged[0].cameras[0].url is None
        assert merged[0].cameras[1].url == "rtsp://10.0.0.21:8555/camera2"

    def test_merge_gives_new_brokers_default_cameras(self):
        broker = Broker(id="si9", name="New", host="10.0.0.99", topic="scenescape/data/camera/#")
        merged = intersection_config.merge_brokers([broker.model_dump()], [])
        assert [c.name for c in merged[0].cameras] == [f"si9-camera{n}" for n in range(1, 5)]
        assert all(c.url is None for c in merged[0].cameras)

    def test_merge_skips_invalid_broker_records(self):
        merged = intersection_config.merge_brokers([{"id": "si1"}], [])
        assert merged == []

