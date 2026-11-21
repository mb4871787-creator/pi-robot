"""
vision_node.py
--------------
Detects QR codes in camera frames using OpenCV's QR detector.
Computes position error and publishes detection events.

Subscribes:
    Topics.CAMERA_FRAME

Publishes:
    Topics.VISION_QR_DETECTED  →  { qr_data, bbox, center_x, center_y,
                                     area, error_x, error_y, frame_w, frame_h }
    Topics.VISION_QR_LOST      →  {}
"""

import cv2
import time
import queue
import numpy as np
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pi_robot.core.base_node import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("vision_node")


class VisionNode(BaseNode):
    """
    Processes camera frames to detect QR codes.

    Detection result includes:
      - Decoded QR data string
      - Bounding box polygon
      - Center pixel coordinates
      - Area (pixels²) — used as proxy for distance
      - Error X/Y relative to frame center — used by PID in decision_node
    """

    def __init__(self, **kwargs):
        super().__init__(name="vision_node", **kwargs)
        self._inbox         = None
        self._detector      = None
        self._was_detected  = False
        self._frame_skip    = cfg("vision", "frame_skip", 2)
        self._min_area      = cfg("vision", "min_qr_area", 500)
        self._frame_count   = 0

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def setup(self) -> None:
        self._inbox    = self.bus.subscribe(Topics.CAMERA_FRAME, maxsize=5)
        self._detector = cv2.QRCodeDetector()
        logger.info("vision_node ready — QR detector initialized")

    def run(self) -> None:
        logger.info("vision_node running")

        while self.running:
            try:
                msg = self._inbox.get(timeout=1.0)
            except queue.Empty:
                continue

            frame = msg["payload"].get("frame")
            if frame is None:
                continue

            # Optionally skip frames for performance
            self._frame_count += 1
            if self._frame_skip > 0 and (self._frame_count % self._frame_skip != 0):
                continue

            self._process_frame(frame)

    # ── Processing ────────────────────────────────────────────────────────────
    def _process_frame(self, frame: np.ndarray) -> None:
        """Detect QR codes and publish results."""
        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect + decode
        data, bbox, _ = self._detector.detectAndDecode(gray)

        if data and bbox is not None:
            # bbox shape: (1, N, 2) — N corners
            pts = bbox[0].astype(int)

            # Compute bounding polygon center
            center_x = float(np.mean(pts[:, 0]))
            center_y = float(np.mean(pts[:, 1]))

            # Compute area of bounding polygon (shoelace formula)
            area = float(cv2.contourArea(pts))

            if area < self._min_area:
                # Too small — probably noise
                logger.debug(f"QR detected but area={area:.0f} < min={self._min_area} — ignored")
                self._publish_lost()
                return

            # Error relative to frame center
            error_x = center_x - frame_cx   # + means QR is to the right
            error_y = center_y - frame_cy   # + means QR is below center

            logger.debug(
                f"QR detected: data='{data}' "
                f"cx={center_x:.1f} cy={center_y:.1f} "
                f"area={area:.0f} "
                f"err_x={error_x:.1f} err_y={error_y:.1f}"
            )

            self.publish(
                Topics.VISION_QR_DETECTED,
                qr_data  = data,
                bbox     = pts.tolist(),
                center_x = center_x,
                center_y = center_y,
                area     = area,
                error_x  = error_x,
                error_y  = error_y,
                frame_w  = frame_w,
                frame_h  = frame_h,
            )
            self._was_detected = True

        else:
            # No QR in this frame
            if self._was_detected:
                self._publish_lost()
            self._was_detected = False

    def _publish_lost(self) -> None:
        """Publish a 'lost' event once when QR disappears."""
        logger.info("QR lost")
        self.publish(Topics.VISION_QR_LOST)
        self._was_detected = False
