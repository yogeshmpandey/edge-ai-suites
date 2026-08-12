# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
import asyncio
import json
import logging
import ssl

import aiomqtt

from config import (
    INTERSECTIONS_CONFIG_PATH,
    MAX_CONCURRENT_EVENTS,
    BROKER_RECONNECT_DELAY,
    NVR_SCENESCAPE_ENABLED,
    SCENESCAPE_MQTT_BROKER,
    SCENESCAPE_MQTT_PORT,
    SCENESCAPE_MQTT_TOPIC,
)
from model.broker import Broker
from service import intersection_config, redis_store
from service.mqtt_listener import handle_frigate_message, handle_scenescape_message

logger = logging.getLogger("broker-manager")

_tasks: dict[str, asyncio.Task] = {}
_semaphore = asyncio.Semaphore(MAX_CONCURRENT_EVENTS)


def _tls_context() -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


async def _dispatch(broker: Broker, payload, topic, state):
    async with _semaphore:
        if broker.type == "frigate":
            await handle_frigate_message(payload, topic)
        else:
            await handle_scenescape_message(payload, topic, state, broker_id=broker.id)


async def _run_broker(broker: Broker):
    state = {"last_processed": 0, "interval": broker.throttle_interval}
    while True:
        try:
            async with aiomqtt.Client(
                hostname=broker.host,
                port=broker.port,
                tls_context=_tls_context() if broker.use_tls else None,
            ) as client:
                await client.subscribe(broker.topic, qos=1)
                logger.info(f"[{broker.id}] subscribed to {broker.topic} at {broker.host}:{broker.port}")
                async for message in client.messages:
                    topic = str(message.topic)
                    try:
                        payload = json.loads(message.payload.decode("utf-8", errors="ignore"))
                        asyncio.create_task(_dispatch(broker, payload, topic, state))
                    except json.JSONDecodeError as e:
                        logger.error(f"[{broker.id}] failed to decode message: {e}")
                    except Exception as e:
                        logger.error(f"[{broker.id}] error processing message: {e}", exc_info=True)
        except aiomqtt.MqttError as e:
            logger.error(f"[{broker.id}] connection error: {e}; reconnecting in {BROKER_RECONNECT_DELAY}s")
            await asyncio.sleep(BROKER_RECONNECT_DELAY)


def _as_broker(broker) -> Broker:
    return broker if isinstance(broker, Broker) else Broker(**broker)


async def start_broker(broker):
    broker = _as_broker(broker)
    await stop_broker(broker.id)
    if not broker.enabled:
        logger.info(f"[{broker.id}] disabled, not starting")
        return
    _tasks[broker.id] = asyncio.create_task(_run_broker(broker))


async def stop_broker(broker_id: str):
    task = _tasks.pop(broker_id, None)
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


async def restart_broker(broker):
    broker = _as_broker(broker)
    await start_broker(broker)


async def start_all_from_redis(request=None):
    for data in await redis_store.get_brokers(request):
        await start_broker(data)


async def stop_all():
    for broker_id in list(_tasks.keys()):
        await stop_broker(broker_id)


async def _has_scenescape(request=None) -> bool:
    return any(b.get("type") == "scenescape" for b in await redis_store.get_brokers(request))


async def load_intersections_config(path: str = INTERSECTIONS_CONFIG_PATH, request=None):
    """Seed Redis from intersections.yaml — the single source of configuration.

    Intersections listed in the file win: brokers in Redis that are no longer in
    the file are stopped and removed. When the file has no entries, the legacy
    ``SCENESCAPE_MQTT_BROKER`` environment variable still seeds a default si1.
    """
    intersections = intersection_config.load_intersections(path)
    if intersections:
        config_ids = {i.id for i in intersections}
        for b in await redis_store.get_brokers(request):
            if b["id"] not in config_ids:
                await stop_broker(b["id"])
                await redis_store.delete_broker(b["id"], request)
        for broker in intersection_config.to_brokers(intersections):
            await redis_store.save_broker(broker.id, broker.model_dump(), request)
        logger.info(f"Loaded {len(intersections)} intersection(s) from {path}")

    if NVR_SCENESCAPE_ENABLED and not await _has_scenescape(request):
        legacy = Broker(
            id="si1",
            name="Smart Intersection 1",
            host=SCENESCAPE_MQTT_BROKER,
            port=SCENESCAPE_MQTT_PORT,
            topic=SCENESCAPE_MQTT_TOPIC,
            type="scenescape",
            use_tls=True,
        )
        await redis_store.save_broker(legacy.id, legacy.model_dump(), request)
        logger.info("Seeded default si1 broker from environment")

    await sync_yaml_from_redis(request=request, path=path)


async def sync_yaml_from_redis(request=None, path: str = INTERSECTIONS_CONFIG_PATH):
    """Persist current Redis brokers to intersections.yaml.

    Camera definitions are not stored in Redis, so they are merged back in from
    the file on disk. Best-effort — errors are logged, not raised.
    """
    try:
        brokers = await redis_store.get_brokers(request)
    except Exception as e:  # noqa: BLE001 - never let a sync failure break a request
        logger.warning(f"Could not read brokers from Redis for YAML sync: {e}")
        return
    existing = intersection_config.load_intersections(path)
    intersection_config.save_intersections(path, intersection_config.merge_brokers(brokers, existing))

