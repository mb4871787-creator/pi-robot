"""
message_bus.py
--------------
Central publish/subscribe message bus for inter-node communication.

Design:
  - Each topic has its own Queue
  - Nodes subscribe to topics they care about
  - Nodes publish to topics by name
  - All operations are thread-safe (Queue is thread-safe by design)

Message Format (dict):
  {
      "topic":     str,   # e.g. "vision/qr_detected"
      "sender":    str,   # node name
      "timestamp": float, # time.time()
      "payload":   dict   # topic-specific data
  }
"""

import queue
import time
import logging
import threading
from typing import Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

# ── Topic name constants (avoids typos across nodes) ─────────────────────────
class Topics:
    # Camera → Vision
    CAMERA_FRAME        = "camera/frame"

    # Vision → Decision
    VISION_QR_DETECTED  = "vision/qr_detected"
    VISION_QR_LOST      = "vision/qr_lost"

    # Decision → Command
    DECISION_ACTION     = "decision/action"

    # Command → Serial
    COMMAND_SERIAL      = "command/serial"

    # Serial → Decision (feedback)
    SERIAL_FEEDBACK     = "serial/feedback"

    # System-wide control
    SYSTEM_MODE         = "system/mode"       # AUTO | MANUAL
    SYSTEM_SHUTDOWN     = "system/shutdown"
    SYSTEM_HEARTBEAT    = "system/heartbeat"

    # Manual control
    MANUAL_COMMAND      = "manual/command"

    # Logging
    LOG_EVENT           = "log/event"


# ── Message factory ───────────────────────────────────────────────────────────
def make_message(topic: str, sender: str, payload: dict) -> dict:
    """Create a standardized message dict."""
    return {
        "topic":     topic,
        "sender":    sender,
        "timestamp": time.time(),
        "payload":   payload,
    }


# ── Message Bus ───────────────────────────────────────────────────────────────
class MessageBus:
    """
    Lightweight publish/subscribe message bus.

    Supports:
      - Multiple subscribers per topic
      - Wildcard subscription via prefix (e.g. "vision/*")
      - Optional callback-based subscription (runs in publisher's thread)
      - Queue-based subscription (polled by subscriber's thread)
    """

    def __init__(self, maxsize: int = 100):
        self._maxsize = maxsize
        self._lock = threading.Lock()

        # topic -> list of Queue objects (one per subscriber)
        self._queues: Dict[str, List[queue.Queue]] = {}

        # topic -> list of callbacks (called synchronously on publish)
        self._callbacks: Dict[str, List[Callable]] = {}

    # ── Subscription ──────────────────────────────────────────────────────────
    def subscribe(self, topic: str, maxsize: int = None) -> queue.Queue:
        """
        Subscribe to a topic. Returns a Queue the caller polls for messages.

        Args:
            topic:   Topic string (exact match or prefix ending with '/*')
            maxsize: Override default queue size

        Returns:
            queue.Queue — poll this in your node's run loop
        """
        q = queue.Queue(maxsize=maxsize if maxsize is not None else self._maxsize)
        with self._lock:
            self._queues.setdefault(topic, []).append(q)
        logger.debug(f"Subscribed queue to topic: {topic}")
        return q

    def subscribe_callback(self, topic: str, callback: Callable) -> None:
        """
        Subscribe with a callback function.
        Callback is called synchronously in the publisher's thread — keep it fast.
        """
        with self._lock:
            self._callbacks.setdefault(topic, []).append(callback)
        logger.debug(f"Subscribed callback to topic: {topic}")

    def unsubscribe(self, topic: str, q: queue.Queue) -> None:
        """Remove a queue subscription."""
        with self._lock:
            if topic in self._queues:
                self._queues[topic].discard(q)

    # ── Publishing ────────────────────────────────────────────────────────────
    def publish(self, message: dict) -> int:
        """
        Publish a message to all subscribers of its topic.

        Returns:
            Number of subscribers delivered to.
        """
        topic = message.get("topic", "")
        delivered = 0

        with self._lock:
            targets_q  = list(self._queues.get(topic, []))
            targets_cb = list(self._callbacks.get(topic, []))

        # Deliver to queue subscribers
        for q in targets_q:
            try:
                q.put_nowait(message)
                delivered += 1
            except queue.Full:
                logger.warning(f"Queue full for topic '{topic}' — message dropped")

        # Deliver to callback subscribers
        for cb in targets_cb:
            try:
                cb(message)
                delivered += 1
            except Exception as e:
                logger.error(f"Callback error on topic '{topic}': {e}")

        return delivered

    # ── Convenience publish ───────────────────────────────────────────────────
    def publish_simple(self, topic: str, sender: str, **payload_kwargs) -> int:
        """Shorthand: bus.publish_simple('vision/qr_detected', 'vision_node', x=10, y=20)"""
        msg = make_message(topic, sender, dict(payload_kwargs))
        return self.publish(msg)

    # ── Diagnostics ───────────────────────────────────────────────────────────
    def topic_info(self) -> dict:
        """Return subscriber counts per topic (for diagnostics)."""
        with self._lock:
            return {
                topic: len(qs)
                for topic, qs in self._queues.items()
            }


# ── Singleton bus instance shared across nodes ────────────────────────────────
_bus: Optional[MessageBus] = None


def get_bus(maxsize: int = 100) -> MessageBus:
    """Return the global MessageBus singleton."""
    global _bus
    if _bus is None:
        _bus = MessageBus(maxsize=maxsize)
        logger.info("Global MessageBus created")
    return _bus
