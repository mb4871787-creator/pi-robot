"""
base_node.py
------------
Abstract base class for all robot nodes.

Each node:
  - Runs in its own daemon thread
  - Has access to the shared MessageBus
  - Has a standard lifecycle: setup() → run() → cleanup()
  - Can be started/stopped cleanly
"""

import threading
import logging
import time
from abc import ABC, abstractmethod
from pi_robot.core.message_bus import MessageBus, get_bus, make_message, Topics


class BaseNode(ABC):
    """
    Abstract base for all nodes in the pi_robot system.

    Subclass and implement:
        setup()    — called once before the run loop (optional)
        run()      — the node's main loop (must check self.running)
        cleanup()  — called once after run() exits (optional)
    """

    def __init__(self, name: str, bus: MessageBus = None):
        self.name    = name
        self.bus     = bus or get_bus()
        self.running = False
        self._thread: threading.Thread = None
        self.logger  = logging.getLogger(name)

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def setup(self) -> None:
        """Override to initialize resources (camera, serial port, etc.)."""
        pass

    @abstractmethod
    def run(self) -> None:
        """
        Main loop — must periodically check `self.running` and exit when False.

        Example:
            while self.running:
                msg = self.inbox.get(timeout=1.0)
                self.process(msg)
        """
        pass

    def cleanup(self) -> None:
        """Override to release resources on shutdown."""
        pass

    # ── Thread management ─────────────────────────────────────────────────────
    def start(self) -> None:
        """Start the node in a background daemon thread."""
        if self._thread and self._thread.is_alive():
            self.logger.warning(f"{self.name} already running — ignoring start()")
            return

        self.running = True
        self._thread = threading.Thread(
            target=self._run_wrapper,
            name=self.name,
            daemon=True,
        )
        self._thread.start()
        self.logger.info(f"{self.name} started (thread: {self._thread.name})")

    def stop(self, timeout: float = 3.0) -> None:
        """Signal the node to stop and wait for its thread to finish."""
        self.logger.info(f"{self.name} stopping…")
        self.running = False
        if self._thread:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                self.logger.warning(f"{self.name} thread did not exit in {timeout}s")

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ── Internal wrapper ──────────────────────────────────────────────────────
    def _run_wrapper(self) -> None:
        try:
            self.setup()
            self.run()
        except Exception as e:
            self.logger.exception(f"Unhandled exception in {self.name}: {e}")
            self._publish_error(str(e))
        finally:
            try:
                self.cleanup()
            except Exception as e:
                self.logger.error(f"Error in {self.name}.cleanup(): {e}")
            self.logger.info(f"{self.name} thread exited")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def publish(self, topic: str, **payload) -> None:
        """Publish a message on the bus."""
        msg = make_message(topic, self.name, dict(payload))
        self.bus.publish(msg)

    def _publish_error(self, error_msg: str) -> None:
        self.publish(
            Topics.LOG_EVENT,
            level="ERROR",
            node=self.name,
            message=error_msg,
        )

    def sleep(self, seconds: float) -> None:
        """Interruptible sleep — stops early if node is told to shut down."""
        deadline = time.time() + seconds
        while self.running and time.time() < deadline:
            time.sleep(0.05)
