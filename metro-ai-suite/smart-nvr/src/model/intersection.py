# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Domain models for the intersection configuration file.

An intersection describes one Smart Intersection (SI) node: its human readable
name, its IP address and the four cameras attached to it. The intersection IP
is used for the SceneScape MQTT broker. Each camera's RTSP source is a fully
self-contained ``url`` — cameras commonly live on different hosts, so there is
no shared default to fall back to beyond a same-host convenience default.
"""

from __future__ import annotations

import re

from pydantic import BaseModel, Field, model_validator

from model.broker import Broker

DEFAULT_MQTT_PORT = 1883
DEFAULT_RTSP_PORT = 8554
DEFAULT_TOPIC = "scenescape/data/camera/#"
CAMERAS_PER_INTERSECTION = 4

_CAMERA_SUFFIX = re.compile(r"^[^-]+-")


class Camera(BaseModel):
    """A single camera of an intersection.

    ``name`` is the Frigate camera name and must be prefixed with the
    intersection id (``si1-camera1``). ``url`` is the full RTSP source URL for
    this camera (e.g. ``rtsp://10.0.0.21:8554/camera1``); when omitted, it
    defaults to the intersection IP on the default RTSP port with a path
    derived from the camera name.
    """

    name: str
    url: str | None = None

    def rtsp_url(self, fallback_ip: str) -> str:
        if self.url:
            return self.url
        path = _CAMERA_SUFFIX.sub("", self.name)
        return f"rtsp://{fallback_ip}:{DEFAULT_RTSP_PORT}/{path}"



class Intersection(BaseModel):
    """One SI node with its MQTT broker settings and its cameras."""

    id: str = ""
    name: str = ""
    ip: str
    mqtt_port: int = DEFAULT_MQTT_PORT
    topic: str = DEFAULT_TOPIC
    use_tls: bool = True
    throttle_interval: float = 2.0
    enabled: bool = True
    cameras: list[Camera] = Field(default_factory=list)

    @model_validator(mode="after")
    def _apply_defaults(self) -> Intersection:
        if not self.id:
            self.id = self._derive_id()
        if not self.name:
            self.name = self.id
        if not self.cameras:
            self.cameras = [
                Camera(name=f"{self.id}-camera{n}")
                for n in range(1, CAMERAS_PER_INTERSECTION + 1)
            ]
        return self

    def _derive_id(self) -> str:
        """Use the prefix of the first camera name (``si1-camera1`` -> ``si1``)."""
        for camera in self.cameras:
            prefix, _, rest = camera.name.partition("-")
            if rest and prefix:
                return prefix
        raise ValueError("intersection requires an 'id' or cameras named '<id>-camera<n>'")

    def to_broker(self) -> Broker:
        """Convert to the MQTT broker record consumed by the broker manager."""
        return Broker(
            id=self.id,
            name=self.name,
            host=self.ip,
            port=self.mqtt_port,
            topic=self.topic,
            type="scenescape",
            use_tls=self.use_tls,
            throttle_interval=self.throttle_interval,
            enabled=self.enabled,
        )

    @classmethod
    def from_broker(cls, broker: Broker, cameras: list[Camera] | None = None) -> Intersection:
        """Build an intersection from a broker record, keeping known cameras."""
        return cls(
            id=broker.id,
            name=broker.name,
            ip=broker.host,
            mqtt_port=broker.port,
            topic=broker.topic,
            use_tls=broker.use_tls,
            throttle_interval=broker.throttle_interval,
            enabled=broker.enabled,
            cameras=list(cameras or []),
        )
