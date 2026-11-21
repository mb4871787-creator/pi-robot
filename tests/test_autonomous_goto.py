"""
test_autonomous_goto.py
------------------------
Tests for the odometry-driven autonomous sequence introduced on top of the
existing pi_robot architecture:

  1. serial_node.py   — parsing of BASE "POS:" / "POS:REACHED:" lines and
                         ARM "SEQ_STEP" / "SEQ_DONE" / "SEQ_ABORTED" lines
                         into structured SERIAL_FEEDBACK messages.
  2. command_node.py   — BASE:GOTO / BASE:POS? / BASE:POS:RESET command
                         building, the BASE:ROTATECW vs BASE:ROTATECCW
                         direction fix, and the SEQ 0/1/2 mapping.
  3. decision_node.py  — a full simulated run of the 11-step AUTO sequence,
                         driving the state machine purely with fake
                         SERIAL_FEEDBACK messages (no real hardware/serial).

Run with:  python -m unittest pi_robot.tests.test_autonomous_goto -v
"""

import sys
import os
import time
import queue
import unittest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from pi_robot.core.message_bus import MessageBus, make_message, Topics
from pi_robot.nodes.serial_node import SerialNode
from pi_robot.nodes.command_node import CommandNode
from pi_robot.nodes.decision_node import DecisionNode, State


# ─────────────────────────────────────────────────────────────────────────────
class TestSerialFeedbackParsing(unittest.TestCase):
    """serial_node._handle_feedback() must understand the new line shapes."""

    def setUp(self):
        self.bus = MessageBus()
        self.node = SerialNode(bus=self.bus)
        self.node._base_ser = MagicMock()
        self.node._arm_ser = MagicMock()
        self.q = self.bus.subscribe(Topics.SERIAL_FEEDBACK, maxsize=50)

    def _last_parsed(self):
        msg = None
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break
        self.assertIsNotNone(msg, "no SERIAL_FEEDBACK message was published")
        return msg["payload"]["parsed"]

    def test_pos_query_reply(self):
        self.node._handle_feedback("POS:0.5012,0.4988,89.7,1.2300", source="base")
        parsed = self._last_parsed()
        self.assertEqual(parsed["type"], "POS")
        self.assertAlmostEqual(parsed["x"], 0.5012, places=4)
        self.assertAlmostEqual(parsed["y"], 0.4988, places=4)
        self.assertAlmostEqual(parsed["heading"], 89.7, places=1)
        self.assertAlmostEqual(parsed["total_distance"], 1.23, places=2)

    def test_pos_reached(self):
        self.node._handle_feedback("POS:REACHED:0.5000,0.0010", source="base")
        parsed = self._last_parsed()
        self.assertEqual(parsed["type"], "POS_REACHED")
        self.assertAlmostEqual(parsed["x"], 0.5000, places=4)
        self.assertAlmostEqual(parsed["y"], 0.0010, places=4)

    def test_seq_done(self):
        self.node._handle_feedback("SEQ_DONE 1", source="arm")
        parsed = self._last_parsed()
        self.assertEqual(parsed["type"], "SEQ_DONE")
        self.assertEqual(parsed["box"], 1)

    def test_seq_aborted(self):
        self.node._handle_feedback("SEQ_ABORTED", source="arm")
        parsed = self._last_parsed()
        self.assertEqual(parsed["type"], "SEQ_ABORTED")

    def test_seq_step(self):
        self.node._handle_feedback("SEQ_STEP 6 box=1 (Gripper close)", source="arm")
        parsed = self._last_parsed()
        self.assertEqual(parsed["type"], "SEQ_STEP")
        self.assertEqual(parsed["step"], 6)
        self.assertEqual(parsed["box"], 1)

    def test_malformed_pos_falls_back_gracefully(self):
        # Should not raise, and should still publish *something*.
        self.node._handle_feedback("POS:not,valid,data", source="base")
        parsed = self._last_parsed()
        self.assertIn("raw", parsed)


# ─────────────────────────────────────────────────────────────────────────────
class TestCommandNodeGoto(unittest.TestCase):
    """command_node._build() for the new BASE:GOTO/POS actions and fixes."""

    def setUp(self):
        self.bus = MessageBus()
        self.node = CommandNode(bus=self.bus)

    def test_goto_basic(self):
        cmd = self.node._build("BASE:GOTO", {"x": 0.5, "y": 0.0})
        self.assertEqual(cmd, "BASE:GOTO:0.5000,0.0000\n")

    def test_goto_negative_diagonal(self):
        cmd = self.node._build("BASE:GOTO", {"x": -0.30, "y": 0.70})
        self.assertEqual(cmd, "BASE:GOTO:-0.3000,0.7000\n")

    def test_goto_with_tolerance(self):
        cmd = self.node._build("BASE:GOTO", {"x": 0.3, "y": 0.0, "tolerance": 0.02})
        self.assertEqual(cmd, "BASE:GOTO:0.3000,0.0000,0.0200\n")

    def test_goto_cancel(self):
        cmd = self.node._build("BASE:GOTO:CANCEL", {})
        self.assertEqual(cmd, "BASE:GOTO:CANCEL\n")

    def test_pos_query(self):
        cmd = self.node._build("BASE:POS?", {})
        self.assertEqual(cmd, "BASE:POS?\n")

    def test_pos_reset(self):
        cmd = self.node._build("BASE:POS:RESET", {})
        self.assertEqual(cmd, "BASE:POS:RESET\n")

    def test_rotate_direction_is_not_flipped(self):
        """
        Regression test: BASE:ROTATECCW must produce BASE:ROTATECCW on the
        wire, not BASE:ROTATECW. Previously ROTATECW/CCW were merged into
        the BASE:TURN branch and always fell back to the "RIGHT" direction
        default, silently flipping every CCW rotation to CW.
        """
        ccw = self.node._build("BASE:ROTATECCW", {})
        cw = self.node._build("BASE:ROTATECW", {})
        self.assertIn("BASE:ROTATECCW", ccw)
        self.assertNotIn("BASE:ROTATECW\n", ccw.replace("BASE:ROTATECCW", ""))
        self.assertIn("BASE:ROTATECW", cw)

    def test_seq_mapping_pick_is_box_0(self):
        self.assertEqual(self.node._build("ARM:PICK", {}), "SEQ 0\n")

    def test_seq_mapping_place1_is_box_1(self):
        self.assertEqual(self.node._build("ARM:PLACE1", {}), "SEQ 1\n")

    def test_seq_mapping_place2_is_box_2(self):
        self.assertEqual(self.node._build("ARM:PLACE2", {}), "SEQ 2\n")

    def test_runseq_explicit_box(self):
        self.assertEqual(self.node._build("ARM:RUNSEQ:2", {}), "SEQ 2\n")


# ─────────────────────────────────────────────────────────────────────────────
class TestAutoSequenceFlow(unittest.TestCase):
    """
    Drive decision_node's AUTO state machine end-to-end using simulated
    SERIAL_FEEDBACK messages — no real serial port or ESP32 required.

    Strategy: step the node's run-loop body manually (the same calls run()
    would make) and inject feedback on the bus exactly as serial_node would,
    based on what action the node just published.
    """

    def setUp(self):
        self.bus = MessageBus()
        self.node = DecisionNode(bus=self.bus)
        self.node.setup()
        self.actions_box = self.bus.subscribe(Topics.DECISION_ACTION, maxsize=200)

    def _publish_feedback(self, parsed, source="base"):
        self.bus.publish(make_message(
            Topics.SERIAL_FEEDBACK, "test", {"raw": str(parsed), "parsed": parsed, "source": source}
        ))

    def _drain_actions(self):
        cmds = []
        while True:
            try:
                msg = self.actions_box.get_nowait()
                cmds.append(msg["payload"]["command"])
            except queue.Empty:
                break
        return cmds

    def _tick(self):
        """
        Advance the node by one logical step.

        decision_node._next() only updates self._state and clears the timer;
        the actual entry action (BASE:GOTO, SEQ, BASE:ROTATECW, ...) is sent
        the *next* time _handle_auto() runs (when it notices _state_start is
        None). The real run() loop calls _handle_auto() continuously, so
        this never matters there — but a single manual tick in a test can
        land exactly on the transition without yet seeing the entry action.
        To keep call sites simple, run the loop body, and if that call
        caused a transition (timer reset to None) without yet issuing the
        entry action, run it once more so the entry action is visible to
        the caller immediately after _tick() returns.
        """
        self._tick_once()
        if self.node._state_start is None and self.node._state not in (
            State.MANUAL, State.PI_SEQ
        ):
            self._tick_once()

    def _tick_once(self):
        self.node._drain_mode()
        self.node._drain_feedback()
        self.node._drain_qr()
        if self.node._state == State.MANUAL:
            self.node._handle_manual()
        elif self.node._state == State.PI_SEQ:
            self.node._handle_manual(allow_abort_only=True)
            self.node._handle_pi_seq()
        else:
            self.node._handle_auto()

    def _start_auto(self):
        self.bus.publish(make_message(Topics.SYSTEM_MODE, "test", {"mode": "AUTO"}))
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_RESET)
        self._drain_actions()  # discard BASE:STOP + BASE:POS:RESET from entry

    def test_full_sequence_happy_path(self):
        self._start_auto()

        # Step 0: AUTO_RESET — settles after 0.3s, no feedback needed.
        time.sleep(0.31)
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_GOTO_1)
        cmds = self._drain_actions()
        self.assertIn("BASE:GOTO", cmds)

        # Step 1: GOTO (0.5, 0.0) — send POS_REACHED.
        self._publish_feedback({"type": "POS_REACHED", "x": 0.50, "y": 0.0})
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_ROTATE_CCW)
        cmds = self._drain_actions()
        self.assertIn("BASE:ROTATECCW", cmds)

        # Step 2: rotate CCW from 0 -> 95 degrees (overshoot slightly past 90).
        for heading in (0.0, 30.0, 60.0, 95.0):
            self._publish_feedback({"type": "POS", "x": 0.5, "y": 0.0,
                                     "heading": heading, "total_distance": 0.5})
            self._tick()
        self.assertEqual(self.node._state, State.AUTO_GOTO_2)
        cmds = self._drain_actions()
        self.assertIn("BASE:STOP", cmds)
        self.assertIn("BASE:GOTO", cmds)

        # Step 3: GOTO (0.5, 0.5)
        self._publish_feedback({"type": "POS_REACHED", "x": 0.50, "y": 0.50})
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_PICK)
        cmds = self._drain_actions()
        self.assertIn("ARM:RUNSEQ:0", cmds)

        # Step 4: pick RED cube -> SEQ_DONE 0
        self._publish_feedback({"type": "SEQ_DONE", "box": 0}, source="arm")
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_GOTO_3)
        cmds = self._drain_actions()
        self.assertIn("BASE:GOTO", cmds)

        # Step 5: GOTO (0.5, 1.0)
        self._publish_feedback({"type": "POS_REACHED", "x": 0.50, "y": 1.00})
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_PLACE_1)
        cmds = self._drain_actions()
        self.assertIn("ARM:RUNSEQ:1", cmds)

        # Step 6: place zone 1 -> SEQ_DONE 1
        self._publish_feedback({"type": "SEQ_DONE", "box": 1}, source="arm")
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_ROTATE_CW)
        cmds = self._drain_actions()
        self.assertIn("BASE:ROTATECW", cmds)

        # Step 7: rotate CW from 90 -> -180 (continuing CW past the wrap).
        # Going CW means heading decreases: 90 -> 0 -> -90 -> wraps to +170 -> ... -> -180
        # We just need accumulated rotation to reach -180 total.
        headings = [90.0, 45.0, 0.0, -45.0, -90.0, -135.0, -179.0]
        for h in headings:
            self._publish_feedback({"type": "POS", "x": 0.0, "y": 1.0,
                                     "heading": h, "total_distance": 1.0})
            self._tick()
        # Not quite -180 of accumulated rotation yet (started at 90, now at -179
        # is only 269 degrees of the 180 needed is already exceeded actually).
        # This drives accumulation past -180, so it should have completed.
        self.assertEqual(self.node._state, State.AUTO_GOTO_4)
        cmds = self._drain_actions()
        self.assertIn("BASE:STOP", cmds)

        # Step 8: GOTO (0.0, 1.0)
        self._publish_feedback({"type": "POS_REACHED", "x": 0.00, "y": 1.00})
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_GOTO_5)

        # Step 9: GOTO (-0.30, 0.70)
        self._publish_feedback({"type": "POS_REACHED", "x": -0.30, "y": 0.70})
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_PLACE_2)
        cmds = self._drain_actions()
        self.assertIn("ARM:RUNSEQ:2", cmds)

        # Step 10: final place -> SEQ_DONE 2
        self._publish_feedback({"type": "SEQ_DONE", "box": 2}, source="arm")
        self._tick()
        # AUTO_STOP has no wait condition (BASE:STOP is sent on entry, then
        # it advances immediately) so a single tick walks straight through
        # to AUTO_DONE — verify BASE:STOP was sent along the way.
        self.assertEqual(self.node._state, State.AUTO_DONE)
        cmds = self._drain_actions()
        self.assertIn("BASE:STOP", cmds)

    def test_goto_timeout_aborts_safely(self):
        self.node._step_timeout_s = 0.05  # shrink timeout for a fast test
        self._start_auto()
        time.sleep(0.31)
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_GOTO_1)
        self._drain_actions()

        # No POS_REACHED ever arrives — wait past the timeout.
        time.sleep(0.06)
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_ABORTED)
        cmds = self._drain_actions()
        self.assertIn("BASE:STOP", cmds)
        self.assertIn("ARM:STOP", cmds)

    def test_seq_aborted_feedback_triggers_abort(self):
        self._start_auto()
        time.sleep(0.31)
        self._tick()  # -> AUTO_GOTO_1
        self._publish_feedback({"type": "POS_REACHED", "x": 0.5, "y": 0.0})
        self._tick()  # -> AUTO_ROTATE_CCW
        for heading in (0.0, 95.0):
            self._publish_feedback({"type": "POS", "x": 0.5, "y": 0.0,
                                     "heading": heading, "total_distance": 0.5})
            self._tick()
        self._publish_feedback({"type": "POS_REACHED", "x": 0.5, "y": 0.5})
        self._tick()  # -> AUTO_PICK
        self._drain_actions()

        self._publish_feedback({"type": "SEQ_ABORTED"}, source="arm")
        self._tick()
        self.assertEqual(self.node._state, State.AUTO_ABORTED)

    def test_manual_mode_switch_back_stops_and_homes(self):
        self._start_auto()
        self.bus.publish(make_message(Topics.SYSTEM_MODE, "test", {"mode": "MANUAL"}))
        self._tick()
        self.assertEqual(self.node._state, State.MANUAL)
        cmds = self._drain_actions()
        self.assertIn("BASE:STOP", cmds)
        self.assertIn("ARM:HOME", cmds)


if __name__ == "__main__":
    unittest.main(verbosity=2)
