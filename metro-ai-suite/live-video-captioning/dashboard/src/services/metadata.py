"""
Metadata file reading utilities for the Live Video Captioning Dashboard.

Provides efficient reading of JSONL metadata files with seek optimization.
"""

import asyncio
from pathlib import Path
from typing import AsyncGenerator, Optional

from src.config import POLL_INTERVAL


def _sync_read_latest_line(path: Path) -> Optional[str]:
    """Synchronously read the latest line from a file using seek optimization.
    
    This function seeks to the end of the file and reads backwards to find
    the last complete line, avoiding the need to read the entire file.
    
    Args:
        path: Path to the file to read.
        
    Returns:
        The last non-empty line, or None if file is empty/missing.
    """
    if not path.exists():
        return None
    
    try:
        with path.open('rb') as f:
            f.seek(0, 2)  # Seek to end
            size = f.tell()
            if size == 0:
                return None
            
            # Read last 8KB (enough for most metadata lines)
            read_size = min(1024, size)
            f.seek(-read_size, 2)
            chunk = f.read().decode('utf-8', errors='replace')
            lines = [line.strip() for line in chunk.strip().split('\n') if line.strip()]
            return lines[-1] if lines else None
    except OSError:
        return None


async def read_latest_line(path: Path) -> Optional[str]:
    """Asynchronously read the latest line from a file.
    
    Uses asyncio.to_thread to avoid blocking the event loop.
    
    Args:
        path: Path to the file to read.
        
    Returns:
        The last non-empty line, or None if file is empty/missing.
    """
    return await asyncio.to_thread(_sync_read_latest_line, path)


async def metadata_generator(path: Path) -> AsyncGenerator[str, None]:
    """Generate SSE events from a metadata file.
    
    Polls the file for new lines and yields them as SSE data events.
    Only yields when content changes to avoid duplicate sends.
    
    Args:
        path: Path to the metadata file.
        
    Yields:
        SSE-formatted strings with metadata content.
    """
    last_payload: Optional[str] = None
    while True:
        latest = await read_latest_line(path)
        if latest and latest != last_payload:
            last_payload = latest
            yield f"data: {latest}\n\n"
        await asyncio.sleep(POLL_INTERVAL)
