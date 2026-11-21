"""
ws_bridge_node.py — WebSocket bridge for pi_robot
===================================================
Runs a WebSocket server on the Pi.  Every message published on the
MessageBus is forwarded to all connected laptop clients as JSON.

This is what the original dashboard.py expected — a ws://<pi>:8765
server — but the Pi never had one.  Add this node to main.py and
use the ORIGINAL dashboard.py (WebSocket version) on the laptop.

Install:  pip install websockets
"""

import asyncio
import base64
import json
import logging
import threading
import time

import cv2
import numpy as np

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
    WS_OK = True
except ImportError:
    WS_OK = False

from pi_robot.core.base_node import BaseNode
from pi_robot.core.message_bus import MessageBus, Topics

logger = logging.getLogger("ws_bridge_node")


class WsBridgeNode(BaseNode):
    """
    Subscribes to all MessageBus topics and forwards them as JSON to
    every connected WebSocket client (the laptop dashboard).

    Config (config.yaml):
        ws_bridge:
          host: "0.0.0.0"
          port: 8765
          jpeg_quality: 60     # camera frame compression (1-95)
          jpeg_width: 640      # resize before sending (0 = no resize)
    """

    def __init__(self, bus: MessageBus, config=None):
        super().__init__(name="ws_bridge_node", bus=bus)
        cfg             = getattr(config, "ws_bridge", None)
        self.host       = getattr(cfg, "host",         "0.0.0.0") if cfg else "0.0.0.0"
        self.port       = int(getattr(cfg, "port",     8765))      if cfg else 8765
        self._quality   = [cv2.IMWRITE_JPEG_QUALITY,
                           int(getattr(cfg, "jpeg_quality", 60))   if cfg else 60]
        self._width     = int(getattr(cfg, "jpeg_width", 640))     if cfg else 640

        self._loop:   asyncio.AbstractEventLoop | None = None
        self._clients: set = set()
        self._clients_lock = asyncio.Lock()   # asyncio lock, used inside coroutines

    # ── BaseNode lifecycle ────────────────────────────────────────────────────

    def setup(self):
        if not WS_OK:
            logger.error("websockets package not installed — pip install websockets")
            return

        # Subscribe to every topic we want to forward
        self.bus.subscribe_callback(Topics.CAMERA_FRAME,       self._on_frame)
        self.bus.subscribe_callback(Topics.SERIAL_FEEDBACK,    self._on_serial)
        self.bus.subscribe_callback(Topics.SYSTEM_MODE,        self._on_mode)
        self.bus.subscribe_callback(Topics.DECISION_ACTION,    self._on_decision)
        self.bus.subscribe_callback(Topics.VISION_QR_DETECTED, self._on_qr_detected)
        self.bus.subscribe_callback(Topics.VISION_QR_LOST,     self._on_qr_lost)

        # Optional topics
        for name, cb in [
            ("IMU_DATA",     self._on_imu),
            ("ENCODER_DATA", self._on_encoders),
        ]:
            if hasattr(Topics, name):
                self.bus.subscribe_callback(getattr(Topics, name), cb)

        # Run the asyncio event loop in this node's thread
        self._loop = asyncio.new_event_loop()
        logger.info("WsBridgeNode setup complete — server will start in run()")

    def run(self):
        if not WS_OK or self._loop is None:
            # Keep thread alive so BaseNode doesn't mark us dead
            while self.running:
                time.sleep(1)
            return

        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    def cleanup(self):
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

    # ── WebSocket server ──────────────────────────────────────────────────────

    async def _serve(self):
        logger.info("WebSocket bridge listening on ws://%s:%d", self.host, self.port)
        async with websockets.serve(self._handler, self.host, self.port,
                                    ping_interval=10, ping_timeout=5):
            while self.running:
                await asyncio.sleep(0.5)

    async def _handler(self, ws: "WebSocketServerProtocol"):
        remote = ws.remote_address
        logger.info("Dashboard connected from %s", remote)
        async with self._clients_lock:
            self._clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            async with self._clients_lock:
                self._clients.discard(ws)
            logger.info("Dashboard disconnected from %s", remote)

    # ── Broadcast helpers ─────────────────────────────────────────────────────

    def _broadcast(self, payload: dict):
        """Thread-safe: schedule a coroutine on the asyncio loop."""
        if self._loop is None or not self._loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(self._send_all(json.dumps(payload)),
                                         self._loop)

    async def _send_all(self, text: str):
        async with self._clients_lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                await ws.send(text)
            except Exception:
                pass   # disconnected clients are removed in _handler

    # ── MessageBus callbacks ──────────────────────────────────────────────────

    def _on_frame(self, msg: dict):
        """Encode camera frame as JPEG, base64 it, and push to clients."""
        frame = msg.get("payload", {}).get("frame") if isinstance(msg, dict) else None
        if frame is None and isinstance(msg, np.ndarray):
            frame = msg   # some publishers pass the array directly

        if frame is None or not isinstance(frame, np.ndarray):
            return

        if self._width > 0:
            h, w = frame.shape[:2]
            if w != self._width:
                scale = self._width / w
                frame = cv2.resize(frame,
                                   (self._width, int(h * scale)),
                                   interpolation=cv2.INTER_LINEAR)

        ok, buf = cv2.imencode(".jpg", frame, self._quality)
        if not ok:
            return

        self._broadcast({
            "topic":    "camera/frame",
            "jpeg_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
        })

    def _on_serial(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        self._broadcast({
            "topic":  "serial/feedback",
            "raw":    payload.get("raw",    ""),
            "parsed": payload.get("parsed", {}),
            "source": payload.get("source", ""),
        })

    def _on_mode(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        mode = payload.get("mode", msg if isinstance(msg, str) else "UNKNOWN")
        self._broadcast({"topic": "system/mode", "mode": mode})

    def _on_decision(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        self._broadcast({
            "topic":  "decision/action",
            "state":  payload.get("state",  ""),
            "action": payload.get("action", ""),
        })

    def _on_qr_detected(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        self._broadcast({
            "topic":    "vision/qr_detected",
            "qr_data":  payload.get("qr_data", ""),
        })

    def _on_qr_lost(self, _):
        self._broadcast({"topic": "vision/qr_lost"})

    def _on_imu(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        self._broadcast({"topic": "imu/data", **payload})

    def _on_encoders(self, msg: dict):
        payload = msg.get("payload", {}) if isinstance(msg, dict) else {}
        self._broadcast({"topic": "encoder/data", **payload})
