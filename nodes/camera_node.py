"""
camera_node.py
--------------
Captures frames from a USB/CSI camera and publishes them on the message bus.

Publishes:
    Topics.CAMERA_FRAME  →  { "frame": np.ndarray }

Config keys used:
    camera.device_index, camera.frame_width, camera.frame_height,
    camera.fps, camera.flip_horizontal, camera.flip_vertical
"""

import cv2
import time
import logging
import sys
import os

# Ensure project root is on path when run directly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pi_robot.core.base_node import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("camera_node")


class CameraNode(BaseNode):
    """Captures video frames and publishes them for downstream nodes."""

    def __init__(self, **kwargs):
        super().__init__(name="camera_node", **kwargs)
        self._cap    = None
        self._fps    = cfg("camera", "fps", 30)
        self._period = 1.0 / self._fps   # seconds per frame

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def setup(self) -> None:
        device   = cfg("camera", "device_index", 0)
        width    = cfg("camera", "frame_width",  640)
        height   = cfg("camera", "frame_height", 480)

        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {device}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._fps)

        self._flip_h = cfg("camera", "flip_horizontal", False)
        self._flip_v = cfg("camera", "flip_vertical",   False)

        logger.info(
            f"Camera opened — device={device} "
            f"res={width}x{height} fps={self._fps}"
        )

    def run(self) -> None:
        logger.info("camera_node running")
        last_time = time.time()

        while self.running:
            ret, frame = self._cap.read()

            if not ret:
                logger.warning("Frame capture failed — retrying…")
                time.sleep(0.5)
                continue

            # Optional flips
            if self._flip_h and self._flip_v:
                frame = cv2.flip(frame, -1)
            elif self._flip_h:
                frame = cv2.flip(frame, 1)
            elif self._flip_v:
                frame = cv2.flip(frame, 0)

            # Publish frame
            self.publish(Topics.CAMERA_FRAME, frame=frame)

            # Throttle to target FPS
            elapsed = time.time() - last_time
            sleep_t = self._period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
            last_time = time.time()

    def cleanup(self) -> None:
        if self._cap:
            self._cap.release()
            logger.info("Camera released")
