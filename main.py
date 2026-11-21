"""
main.py — pi_robot entry point

Usage:
    python main.py --mode manual   # Start in MANUAL mode
    python main.py --mode auto     # Start in AUTO vision mode
    python main.py                 # Use config.yaml mode setting
"""

import sys
import os
import time
import signal
import logging
import argparse

sys.path.insert(0, os.path.dirname(__file__))

from nodes.logger_node        import setup_logging
from config.config_loader     import get as cfg

setup_logging()
logger = logging.getLogger("main")

from pi_robot.core.message_bus         import MessageBus, Topics
from nodes.camera_vision_node import CameraVisionNode
from nodes.decision_node      import DecisionNode
from nodes.command_node       import CommandNode
from nodes.serial_node        import SerialNode
from nodes.logger_node        import LoggerNode
from nodes.manual_node        import ManualNode
from nodes.ps4_node           import PS4Node
from nodes.dashboard_node     import DashboardNode
from nodes.ws_bridge_node     import WsBridgeNode


class RobotController:

    def __init__(self, mode: str = None):
        self._bus   = MessageBus()
        self._nodes = []
        self._mode  = mode or cfg("system", "mode", "MANUAL")
        logger.info(f"RobotController init — mode={self._mode}")

    def build_nodes(self):
        self._camera_vision = CameraVisionNode(bus=self._bus)
        self._decision      = DecisionNode(bus=self._bus)
        self._command       = CommandNode(bus=self._bus)
        self._serial        = SerialNode(bus=self._bus)
        self._log_node      = LoggerNode(bus=self._bus)
        self._manual        = ManualNode(bus=self._bus)
        self._ps4           = PS4Node(bus=self._bus)
        self._dashboard     = DashboardNode(bus=self._bus)
        self._ws_bridge     = WsBridgeNode(bus=self._bus)

        self._nodes = [
            self._camera_vision,
            self._decision,
            self._command,
            self._serial,
            self._log_node,
            self._manual,
            self._ps4,
            self._dashboard,
            self._ws_bridge,
        ]
        logger.info(f"Built {len(self._nodes)} nodes")

    def start(self):
        logger.info("=" * 60)
        logger.info("  pi_robot starting up")
        logger.info(f"  Mode: {self._mode.upper()}")
        logger.info("=" * 60)

        for node in self._nodes:
            node.start()

        # Give nodes time to initialize
        time.sleep(0.5)

        # Override mode from command line
        if self._mode.upper() != cfg("system", "mode", "MANUAL").upper():
            self._bus.publish_simple(
                Topics.SYSTEM_MODE, "main",
                mode=self._mode.upper()
            )

        logger.info("All nodes started")
        logger.info("=" * 60)
        logger.info("  pi_robot — System Status")
        logger.info("=" * 60)
        for node in self._nodes:
            status = "[✓ running]" if node.is_alive() else "[✗ stopped]"
            logger.info(f"  {status}  {node.name}")
        logger.info("=" * 60)
        logger.info(f"  Mode: {self._mode.upper()}")
        logger.info("  Press Ctrl+C to stop")
        logger.info("=" * 60)

    def run(self):
        shutdown_q = self._bus.subscribe(Topics.SYSTEM_SHUTDOWN, maxsize=1)

        try:
            while True:
                # Check for shutdown signal
                try:
                    shutdown_q.get_nowait()
                    logger.info("Shutdown signal received")
                    break
                except Exception:
                    pass

                # Update dashboard node statuses
                for node in self._nodes:
                    if hasattr(self._dashboard, 'update_node_status'):
                        status = "running" if node.is_alive() else "stopped"
                        self._dashboard.update_node_status(node.name, status)

                # Warn about dead nodes
                for node in self._nodes:
                    if not node.is_alive() and node.name != "manual_node":
                        logger.warning(f"Node '{node.name}' is not alive!")

                time.sleep(1.0)

        except KeyboardInterrupt:
            logger.info("Keyboard interrupt — shutting down")

    def stop(self):
        logger.info("Stopping all nodes...")
        timeout = cfg("system", "shutdown_timeout", 3.0)
        for node in reversed(self._nodes):
            node.stop()
        for node in self._nodes:
            node.join(timeout=timeout)
        logger.info("All nodes stopped")


def main():
    parser = argparse.ArgumentParser(description="pi_robot controller")
    parser.add_argument("--mode", choices=["manual","auto","MANUAL","AUTO"],
                        default=None, help="Operating mode")
    args = parser.parse_args()

    mode = args.mode.upper() if args.mode else None

    controller = RobotController(mode=mode)
    controller.build_nodes()
    controller.start()

    def _sig(sig, frame):
        logger.info("Signal received — shutting down")
        controller.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    try:
        controller.run()
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
