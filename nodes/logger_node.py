"""
logger_node.py
--------------
Centralized logging node that subscribes to system log events
and writes structured logs to rotating files.

Subscribes:
    Topics.LOG_EVENT      →  { level, node, message }
    Topics.DECISION_ACTION (for state machine audit trail)
    Topics.SERIAL_FEEDBACK (for ESP32 communication log)
"""

import queue
import logging
import logging.handlers
import time
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pi_robot.core.base_node   import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("logger_node")


def setup_logging() -> None:
    """
    Configure the root Python logger.
    Called once at startup (from main.py) before nodes start.
    """
    log_cfg     = cfg("logging")
    level_str   = log_cfg.get("level", "DEBUG")
    log_to_file = log_cfg.get("log_to_file", True)
    log_dir     = log_cfg.get("log_dir", "logs")
    max_mb      = log_cfg.get("max_file_size_mb", 10)
    backups     = log_cfg.get("backup_count", 5)
    console     = log_cfg.get("console_output", True)

    level = getattr(logging, level_str.upper(), logging.DEBUG)
    root  = logging.getLogger()
    root.setLevel(level)

    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)-8s] %(name)-20s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setFormatter(fmt)
        root.addHandler(ch)

    if log_to_file:
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, "pi_robot.log")
        fh = logging.handlers.RotatingFileHandler(
            log_path,
            maxBytes=max_mb * 1024 * 1024,
            backupCount=backups,
        )
        fh.setFormatter(fmt)
        root.addHandler(fh)
        logger.info(f"File logging → {log_path}")


class LoggerNode(BaseNode):
    """
    Optional node that listens to structured bus events and writes
    a dedicated audit/activity log separate from the Python log stream.
    """

    def __init__(self, **kwargs):
        super().__init__(name="logger_node", **kwargs)
        self._inboxes = {}
        self._activity_log = None

    def setup(self) -> None:
        self._inboxes = {
            "log_event":  self.bus.subscribe(Topics.LOG_EVENT,       maxsize=100),
            "decision":   self.bus.subscribe(Topics.DECISION_ACTION,  maxsize=100),
            "feedback":   self.bus.subscribe(Topics.SERIAL_FEEDBACK,  maxsize=100),
        }

        log_dir = cfg("logging", "log_dir", "logs")
        os.makedirs(log_dir, exist_ok=True)
        activity_path = os.path.join(log_dir, "activity.log")

        self._activity_log = logging.getLogger("activity")
        self._activity_log.setLevel(logging.DEBUG)
        fh = logging.FileHandler(activity_path)
        fh.setFormatter(
            logging.Formatter("%(asctime)s %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
        )
        self._activity_log.addHandler(fh)
        self._activity_log.propagate = False

        logger.info(f"logger_node ready — activity log: {activity_path}")

    def run(self) -> None:
        logger.info("logger_node running")
        while self.running:
            processed = 0
            for name, q in self._inboxes.items():
                try:
                    msg = q.get_nowait()
                    self._log_message(name, msg)
                    processed += 1
                except queue.Empty:
                    pass

            if processed == 0:
                time.sleep(0.05)

    def _log_message(self, source: str, msg: dict) -> None:
        ts      = msg.get("timestamp", time.time())
        sender  = msg.get("sender", "?")
        payload = msg.get("payload", {})

        if source == "log_event":
            level   = payload.get("level", "INFO")
            message = payload.get("message", "")
            self._activity_log.info(f"[{level}] {sender}: {message}")

        elif source == "decision":
            cmd   = payload.get("command", "")
            state = payload.get("state", "")
            self._activity_log.info(f"[ACTION] state={state} cmd={cmd}")

        elif source == "feedback":
            raw = payload.get("raw", "")
            self._activity_log.info(f"[ESP32 ←] {raw}")
