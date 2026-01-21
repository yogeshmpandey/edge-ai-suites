# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""
Target Service Poller for active metrics collection.

This module provides functionality to actively poll a target service's
metrics endpoint and collect data. This is useful when the metrics service
needs to gather metrics from services that don't push data.
"""

import asyncio
import logging
from typing import Optional, Callable, Any

import httpx

from .config import TARGET_SERVICE_URL, METRICS_ENDPOINT, POLL_INTERVAL_SECONDS

logger = logging.getLogger("live-metrics-service.poller")


class TargetServicePoller:
    """
    Polls a target service's metrics endpoint at regular intervals.
    
    This can be used to actively collect metrics from services that expose
    REST API endpoints for metrics data.
    """

    def __init__(
        self,
        target_url: str = TARGET_SERVICE_URL,
        endpoint: str = METRICS_ENDPOINT,
        poll_interval: int = POLL_INTERVAL_SECONDS,
    ):
        self.target_url = target_url.rstrip("/") if target_url else ""
        self.endpoint = endpoint
        self.poll_interval = poll_interval
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._client: Optional[httpx.AsyncClient] = None
        self._callbacks: list[Callable[[dict], Any]] = []
        self._last_metrics: Optional[dict] = None

    @property
    def full_url(self) -> str:
        """Get the full URL for the metrics endpoint."""
        if not self.target_url:
            return ""
        return f"{self.target_url}{self.endpoint}"

    @property
    def is_configured(self) -> bool:
        """Check if the poller is properly configured."""
        return bool(self.target_url)

    @property
    def is_running(self) -> bool:
        """Check if the poller is currently running."""
        return self._running

    @property
    def last_metrics(self) -> Optional[dict]:
        """Get the last collected metrics."""
        return self._last_metrics

    def register_callback(self, callback: Callable[[dict], Any]) -> None:
        """
        Register a callback to be called when new metrics are received.
        
        Args:
            callback: Function that takes a dict of metrics as argument
        """
        self._callbacks.append(callback)

    def unregister_callback(self, callback: Callable[[dict], Any]) -> None:
        """Remove a previously registered callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    async def _poll_once(self) -> Optional[dict]:
        """
        Poll the target service once and return the metrics.
        
        Returns:
            Dict of metrics if successful, None otherwise
        """
        if not self._client or not self.is_configured:
            return None

        try:
            response = await self._client.get(self.full_url, timeout=10.0)
            response.raise_for_status()
            metrics = response.json()
            self._last_metrics = metrics
            
            # Notify all callbacks
            for callback in self._callbacks:
                try:
                    result = callback(metrics)
                    if asyncio.iscoroutine(result):
                        await result
                except Exception as e:
                    logger.error(f"Error in callback: {e}")
            
            return metrics
        except httpx.HTTPStatusError as e:
            logger.warning(f"HTTP error polling {self.full_url}: {e.response.status_code}")
        except httpx.RequestError as e:
            logger.warning(f"Request error polling {self.full_url}: {e}")
        except Exception as e:
            logger.error(f"Unexpected error polling {self.full_url}: {e}")
        
        return None

    async def _poll_loop(self) -> None:
        """Main polling loop."""
        logger.info(f"Starting polling loop for {self.full_url} every {self.poll_interval}s")
        
        while self._running:
            await self._poll_once()
            await asyncio.sleep(self.poll_interval)

    async def start(self) -> None:
        """Start the polling loop."""
        if not self.is_configured:
            logger.info("Poller not configured (no TARGET_SERVICE_URL), skipping")
            return

        if self._running:
            logger.warning("Poller already running")
            return

        self._running = True
        self._client = httpx.AsyncClient()
        self._task = asyncio.create_task(self._poll_loop())
        logger.info(f"Poller started for {self.full_url}")

    async def stop(self) -> None:
        """Stop the polling loop."""
        self._running = False
        
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        
        if self._client:
            await self._client.aclose()
            self._client = None
        
        logger.info("Poller stopped")

    async def poll_now(self) -> Optional[dict]:
        """
        Perform an immediate poll regardless of the interval.
        
        Returns:
            Dict of metrics if successful, None otherwise
        """
        if not self._client:
            self._client = httpx.AsyncClient()
        
        return await self._poll_once()


# Global poller instance
_poller: Optional[TargetServicePoller] = None


def get_poller() -> TargetServicePoller:
    """Get or create the global poller instance."""
    global _poller
    if _poller is None:
        _poller = TargetServicePoller()
    return _poller


async def start_poller() -> None:
    """Start the global poller."""
    poller = get_poller()
    await poller.start()


async def stop_poller() -> None:
    """Stop the global poller."""
    global _poller
    if _poller:
        await _poller.stop()
        _poller = None
