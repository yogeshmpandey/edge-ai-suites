# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Read and write the intersections configuration file.

``intersections.yaml`` is the single source of configuration for a SceneScape
deployment: it holds every intersection (name + IP) and its four cameras. The
setup script uses it to generate the Frigate camera blocks, and the backend uses
it to seed one MQTT broker connection per intersection.
"""

from __future__ import annotations

import logging
import os

import yaml

from model.broker import Broker
from model.intersection import Camera, Intersection

logger = logging.getLogger("intersection-config")

FILE_HEADER = (
    "# Smart NVR intersections - single source of configuration.\n"
    "# Each entry holds the intersection name, its IP (MQTT broker + RTSP source)\n"
    "# and its 4 cameras (<id>-camera1 .. <id>-camera4).\n"
    "# Managed by setup.sh and the /brokers/ API; safe to edit while stopped.\n"
)


def load_intersections(path: str) -> list[Intersection]:
    """Parse ``path`` and return its intersections.

    Returns an empty list when the file is missing or holds no entries. Invalid
    entries are skipped with a warning so a single typo cannot take the whole
    deployment down.
    """
    if not os.path.exists(path):
        logger.info(f"No intersections config at {path}")
        return []

    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        logger.error(f"Could not read intersections config at {path}: {e}")
        return []

    intersections = []
    for entry in data.get("intersections") or []:
        try:
            intersections.append(Intersection(**entry))
        except (TypeError, ValueError) as e:
            logger.warning(f"Skipping invalid intersection entry {entry!r}: {e}")
    return intersections


def save_intersections(path: str, intersections: list[Intersection]) -> bool:
    """Write ``intersections`` to ``path``. Returns False when the write fails."""
    ordered = sorted(intersections, key=lambda i: i.id)
    payload = {"intersections": [i.model_dump(exclude_none=True) for i in ordered]}
    try:
        with open(path, "w") as f:
            f.write(FILE_HEADER)
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    except OSError as e:
        logger.warning(f"Could not write intersections config at {path}: {e}")
        return False
    logger.info(f"Synced {len(ordered)} intersection(s) to {path}")
    return True


def to_brokers(intersections: list[Intersection]) -> list[Broker]:
    return [i.to_broker() for i in intersections]


def cameras_by_id(intersections: list[Intersection]) -> dict[str, list[Camera]]:
    return {i.id: i.cameras for i in intersections}


def merge_brokers(brokers: list[dict], existing: list[Intersection]) -> list[Intersection]:
    """Rebuild intersections from live broker records.

    Camera definitions are not part of a broker record, so they are carried over
    from ``existing`` by intersection id verbatim — each camera's RTSP ``url``
    is self-contained and independent of the MQTT broker host, so no
    re-pointing is needed when the broker host changes. Brokers added through
    the API get the default four cameras (pointing at the broker host as a
    same-host convenience default).
    """
    known = {i.id: i for i in existing}
    merged = []
    for broker in brokers:
        try:
            record = Broker(**broker)
        except (TypeError, ValueError) as e:
            logger.warning(f"Skipping invalid broker record {broker!r}: {e}")
            continue
        previous = known.get(record.id)
        cameras = list(previous.cameras) if previous else []
        merged.append(Intersection.from_broker(record, cameras))
    return merged

