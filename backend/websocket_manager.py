"""
websocket_manager.py
====================
Manages WebSocket connections for IDR-X live navigation streaming.
Supports multiple concurrent clients (browser tabs / debug tools).
"""

import json
import logging
from typing import Any, Dict, List, Set

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ConnectionManager:
    """Thread-safe WebSocket connection registry with broadcast support."""

    def __init__(self) -> None:
        self._active: List[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._active.append(ws)
        logger.debug("WS client connected (%d total)", len(self._active))

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self._active:
            self._active.remove(ws)
        logger.debug("WS client disconnected (%d remaining)", len(self._active))

    async def send(self, ws: WebSocket, payload: Dict[str, Any]) -> None:
        """Send a JSON payload to a single client, silently dropping on error."""
        try:
            await ws.send_text(json.dumps(payload))
        except Exception as exc:
            logger.debug("WS send failed: %s", exc)
            self.disconnect(ws)

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        """Broadcast a JSON payload to all connected clients."""
        dead: List[WebSocket] = []
        for ws in list(self._active):
            try:
                await ws.send_text(json.dumps(payload))
            except Exception as exc:
                logger.debug("WS broadcast failed for client: %s", exc)
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


# Singleton shared across the application
connection_manager = ConnectionManager()
