# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Tests for loading brokers from intersections.yaml and syncing back to it."""

import pytest
import yaml

from service import broker_manager, intersection_config

CONFIG = """
intersections:
  - id: si1
    name: Main Street and 1st Ave
    ip: 10.0.0.11
    cameras:
      - name: si1-camera1
      - name: si1-camera2
      - name: si1-camera3
      - name: si1-camera4
  - id: si2
    name: Broadway and 5th
    ip: 10.0.0.12
    mqtt_port: 8883
    enabled: false
"""


class FakeRedisStore:
    """Minimal stand-in for service.redis_store used by the broker manager."""

    def __init__(self, brokers=None):
        self.brokers = dict(brokers or {})

    async def get_brokers(self, request=None):
        return list(self.brokers.values())

    async def save_broker(self, broker_id, data, request=None):
        self.brokers[broker_id] = data

    async def delete_broker(self, broker_id, request=None):
        return self.brokers.pop(broker_id, None) is not None


@pytest.fixture
def store(monkeypatch):
    fake = FakeRedisStore()
    monkeypatch.setattr(broker_manager, "redis_store", fake)
    return fake


@pytest.fixture
def config_path(tmp_path):
    path = tmp_path / "intersections.yaml"
    path.write_text(CONFIG)
    return str(path)


@pytest.fixture(autouse=True)
def no_broker_tasks(monkeypatch):
    async def noop(*args, **kwargs):
        return None

    monkeypatch.setattr(broker_manager, "stop_broker", noop)


@pytest.mark.asyncio
async def test_intersections_seed_one_broker_each(store, config_path, monkeypatch):
    monkeypatch.setattr(broker_manager, "NVR_SCENESCAPE_ENABLED", False)

    await broker_manager.load_intersections_config(path=config_path)

    assert sorted(store.brokers) == ["si1", "si2"]
    assert store.brokers["si1"]["host"] == "10.0.0.11"
    assert store.brokers["si2"]["port"] == 8883
    assert store.brokers["si2"]["enabled"] is False


@pytest.mark.asyncio
async def test_brokers_absent_from_config_are_removed(store, config_path, monkeypatch):
    monkeypatch.setattr(broker_manager, "NVR_SCENESCAPE_ENABLED", False)
    store.brokers["si9"] = {
        "id": "si9",
        "name": "stale",
        "host": "10.0.0.99",
        "topic": "scenescape/data/camera/#",
    }

    await broker_manager.load_intersections_config(path=config_path)

    assert "si9" not in store.brokers


@pytest.mark.asyncio
async def test_empty_config_seeds_legacy_env_broker(store, tmp_path, monkeypatch):
    path = str(tmp_path / "intersections.yaml")
    open(path, "w").write("intersections: []\n")
    monkeypatch.setattr(broker_manager, "NVR_SCENESCAPE_ENABLED", True)
    monkeypatch.setattr(broker_manager, "SCENESCAPE_MQTT_BROKER", "10.5.5.5")

    await broker_manager.load_intersections_config(path=path)

    assert store.brokers["si1"]["host"] == "10.5.5.5"
    # the seeded broker is written back to the config file
    written = yaml.safe_load(open(path))["intersections"]
    assert written[0]["id"] == "si1"
    assert [c["name"] for c in written[0]["cameras"]] == [f"si1-camera{n}" for n in range(1, 5)]


@pytest.mark.asyncio
async def test_sync_preserves_camera_definitions(store, config_path, monkeypatch):
    monkeypatch.setattr(broker_manager, "NVR_SCENESCAPE_ENABLED", False)
    await broker_manager.load_intersections_config(path=config_path)

    store.brokers["si1"]["name"] = "Renamed intersection"
    await broker_manager.sync_yaml_from_redis(path=config_path)

    reloaded = intersection_config.load_intersections(config_path)
    si1 = next(i for i in reloaded if i.id == "si1")
    assert si1.name == "Renamed intersection"
    assert [c.name for c in si1.cameras] == [f"si1-camera{n}" for n in range(1, 5)]


@pytest.mark.asyncio
async def test_sync_failure_is_not_raised(store, config_path, monkeypatch):
    async def boom(request=None):
        raise RuntimeError("redis down")

    monkeypatch.setattr(store, "get_brokers", boom)
    await broker_manager.sync_yaml_from_redis(path=config_path)
