"""
test_core.py
------------
Unit tests for core components: MessageBus, PIDController, BaseNode.
Run with: python -m pytest tests/ -v
"""

import sys
import os
import time
import queue
import threading
import unittest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pi_robot.core.message_bus   import MessageBus, make_message, Topics
from pi_robot.core.pid_controller import PIDController
from pi_robot.core.base_node     import BaseNode
from pi_robot.nodes.serial_node   import SerialNode


# ─────────────────────────────────────────────────────────────────────────────
class TestMessageBus(unittest.TestCase):

    def setUp(self):
        self.bus = MessageBus(maxsize=10)

    def test_publish_subscribe(self):
        """Basic pub/sub round trip."""
        q = self.bus.subscribe("test/topic")
        msg = make_message("test/topic", "test", {"value": 42})
        self.bus.publish(msg)

        received = q.get(timeout=1.0)
        self.assertEqual(received["payload"]["value"], 42)
        self.assertEqual(received["topic"], "test/topic")
        self.assertEqual(received["sender"], "test")

    def test_multiple_subscribers(self):
        """Multiple queues receive the same message."""
        q1 = self.bus.subscribe("shared/topic")
        q2 = self.bus.subscribe("shared/topic")

        msg = make_message("shared/topic", "src", {"x": 1})
        delivered = self.bus.publish(msg)

        self.assertEqual(delivered, 2)
        self.assertEqual(q1.get_nowait()["payload"]["x"], 1)
        self.assertEqual(q2.get_nowait()["payload"]["x"], 1)

    def test_full_queue_drops_message(self):
        """Full queues drop messages without raising exceptions."""
        q = self.bus.subscribe("drop/topic", maxsize=1)

        # Fill the queue
        self.bus.publish(make_message("drop/topic", "s", {"n": 1}))
        # This should be dropped (queue full)
        delivered = self.bus.publish(make_message("drop/topic", "s", {"n": 2}))

        self.assertEqual(delivered, 0)   # dropped

    def test_publish_simple(self):
        """Convenience publish_simple method."""
        q = self.bus.subscribe("simple/topic")
        self.bus.publish_simple("simple/topic", "node", key="val")
        msg = q.get(timeout=1.0)
        self.assertEqual(msg["payload"]["key"], "val")

    def test_callback_subscriber(self):
        """Callback subscribers are called synchronously."""
        received = []
        self.bus.subscribe_callback("cb/topic", lambda m: received.append(m))
        self.bus.publish(make_message("cb/topic", "src", {"data": "hello"}))
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0]["payload"]["data"], "hello")

    def test_topic_info(self):
        """topic_info returns subscriber counts."""
        self.bus.subscribe("info/a")
        self.bus.subscribe("info/a")
        self.bus.subscribe("info/b")
        info = self.bus.topic_info()
        self.assertEqual(info.get("info/a"), 2)
        self.assertEqual(info.get("info/b"), 1)


# ─────────────────────────────────────────────────────────────────────────────
class TestPIDController(unittest.TestCase):

    def test_proportional_only(self):
        """With ki=kd=0, output = kp * error."""
        pid = PIDController(kp=0.5, ki=0.0, kd=0.0, max_output=100)
        out = pid.compute(setpoint=0, measurement=10)
        # error = 0 - 10 = -10, output = 0.5 * -10 = -5
        self.assertAlmostEqual(out, -5.0, places=1)

    def test_output_clamping(self):
        """Output is clamped to [-max_output, +max_output]."""
        pid = PIDController(kp=100, ki=0, kd=0, max_output=50)
        out = pid.compute(setpoint=0, measurement=1)
        self.assertEqual(out, -50.0)

    def test_reset_clears_state(self):
        """Reset clears integrator and derivative state."""
        pid = PIDController(kp=0.1, ki=0.5, kd=0.0, max_output=100)
        pid.compute(setpoint=100, measurement=0)   # builds up integral
        pid.reset()
        # After reset, integral should be zero — output should be proportional only
        out = pid.compute(setpoint=0, measurement=10)
        self.assertAlmostEqual(out, -1.0, places=0)

    def test_drives_toward_setpoint(self):
        """PID output sign should push measurement toward setpoint."""
        pid = PIDController(kp=1.0, ki=0, kd=0, max_output=1000)
        # measurement > setpoint → output should be negative (decrease measurement)
        out = pid.compute(setpoint=0, measurement=5)
        self.assertLess(out, 0)

        # measurement < setpoint → output should be positive
        out2 = pid.compute(setpoint=10, measurement=0)
        self.assertGreater(out2, 0)


# ─────────────────────────────────────────────────────────────────────────────
class TestBaseNode(unittest.TestCase):

    def test_node_starts_and_stops(self):
        """A concrete node can start and stop cleanly."""

        class CounterNode(BaseNode):
            count = 0
            def run(self):
                while self.running:
                    self.count += 1
                    time.sleep(0.01)

        bus  = MessageBus()
        node = CounterNode(name="counter", bus=bus)
        node.start()
        time.sleep(0.15)
        node.stop(timeout=2.0)

        self.assertFalse(node.is_alive())
        self.assertGreater(node.count, 5)

    def test_node_publishes(self):
        """BaseNode.publish() delivers messages to the bus."""

        class PublishNode(BaseNode):
            def run(self):
                self.publish("test/output", data="hello")
                self.running = False

        bus  = MessageBus()
        q    = bus.subscribe("test/output")
        node = PublishNode(name="pub_test", bus=bus)
        node.start()
        node._thread.join(timeout=2.0)

        msg = q.get(timeout=1.0)
        self.assertEqual(msg["payload"]["data"], "hello")


class TestSerialNode(unittest.TestCase):

    def test_route_arm_command_without_prefix(self):
        """The serial node should pass through raw arm commands correctly."""
        bus = MessageBus()
        node = SerialNode(bus=bus)
        node._arm_ser = MagicMock()
        node._base_ser = MagicMock()

        node._route("HOME")
        node._arm_ser.send.assert_called_with("HOME")

        node._route("SEQ 1")
        node._arm_ser.send.assert_called_with("SEQ 1")


if __name__ == "__main__":
    unittest.main(verbosity=2)
