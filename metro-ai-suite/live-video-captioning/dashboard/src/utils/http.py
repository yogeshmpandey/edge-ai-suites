"""
HTTP utilities for the Live Video Captioning Dashboard.

Provides HTTPException for structured error handling and helper functions
for HTTP operations.
"""

import asyncio
from typing import Any, Optional

import aiohttp
from aiohttp import web


class HTTPException(Exception):
    """HTTP exception for error handling in aiohttp routes.
    
    Attributes:
        status_code: The HTTP status code.
        detail: Error details (string or dict).
    """
    def __init__(self, status_code: int, detail: Any = None) -> None:
        self.status_code = status_code
        self.detail = detail
        super().__init__(str(detail))


def error_response(status: int, message: str, **context: Any) -> web.Response:
    """Create a JSON error response with consistent format.
    
    Args:
        status: HTTP status code.
        message: Error message.
        **context: Additional context fields to include.
        
    Returns:
        aiohttp Response with JSON error body.
    """
    return web.json_response(
        {"error": message, "status": status, **context},
        status=status
    )


async def http_json_async(
    session: aiohttp.ClientSession,
    method: str,
    url: str,
    payload: Optional[dict[str, Any]] = None,
    timeout: Optional[int] = None
) -> str:
    """Perform an async HTTP request using aiohttp.
    
    Args:
        session: aiohttp ClientSession to use.
        method: HTTP method (GET, POST, DELETE, etc.).
        url: Target URL.
        payload: Optional JSON payload for POST/PUT requests.
        timeout: Optional request timeout in seconds.
        
    Returns:
        Response body as string.
        
    Raises:
        HTTPException: On HTTP errors, unreachable server, or timeout.
    """
    headers = {"Accept": "application/json"}
    
    kwargs: dict[str, Any] = {"headers": headers}
    if payload is not None:
        kwargs["json"] = payload
        headers["Content-Type"] = "application/json"
    
    if timeout is not None:
        kwargs["timeout"] = aiohttp.ClientTimeout(total=timeout)
    
    try:
        async with session.request(method, url, **kwargs) as resp:
            body = await resp.text()
            if resp.status >= 400:
                raise HTTPException(
                    status_code=502,
                    detail={
                        "message": "Pipeline server error",
                        "status": resp.status,
                        "body": body
                    }
                )
            return body
    except aiohttp.ClientError as err:
        raise HTTPException(
            status_code=502,
            detail={"message": "Pipeline server unreachable", "error": str(err)}
        )
    except asyncio.TimeoutError:
        raise HTTPException(
            status_code=504,
            detail={"message": "Pipeline server timeout"}
        )


async def create_http_session() -> aiohttp.ClientSession:
    """Create an aiohttp session with connection pooling.
    
    Uses a TCPConnector with increased limits to support many concurrent
    pipeline connections.
    
    Returns:
        Configured aiohttp ClientSession.
    """
    timeout = aiohttp.ClientTimeout(total=120, connect=10)
    connector = aiohttp.TCPConnector(
        limit=200,           # Total connection limit
        limit_per_host=50,   # Connections per host
        ttl_dns_cache=300,   # DNS cache TTL
        enable_cleanup_closed=True,
    )
    return aiohttp.ClientSession(timeout=timeout, connector=connector)
