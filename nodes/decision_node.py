"""
decision_node.py
----------------
Two-phase robot controller:

MANUAL phase (PS4 / keyboard):
  - Drive to cube location
  - Scan QR to identify color (RED/GREEN/BLUE)
  - Pick cube manually
  - Drive to starting point (0,0)
  - Press OPTIONS (PS4) or M (keyboard) to start AUTO

AUTO phase (fully autonomous, odometry-driven, no controller needed):
  Follows the 11-step autonomous spec exactly, using the ESP32 base's
  closed-loop position controller (BASE:GOTO) and odometry feed
  (BASE:POS?) rather than fixed-duration dead reckoning:

    0.  BASE:POS:RESET                      — zero odometry
    1.  BASE:GOTO:0.50,0.00   → POS:REACHED — move forward
    2.  BASE:ROTATECCW, poll BASE:POS?       — rotate until heading ≥ +90°, then BASE:STOP
    3.  BASE:GOTO:0.50,0.50   → POS:REACHED — move to object
    4.  SEQ 0                 → SEQ_DONE 0  — pick RED cube
    5.  BASE:GOTO:0.50,1.00   → POS:REACHED — move to drop zone
    6.  SEQ 1                 → SEQ_DONE 1  — place object (zone 1)
    7.  BASE:ROTATECW, poll BASE:POS?        — rotate until heading ≤ -180° (wrapped), then BASE:STOP
    8.  BASE:GOTO:0.00,1.00   → POS:REACHED — move forward
    9.  BASE:GOTO:-0.30,0.70  → POS:REACHED — move diagonal
    10. SEQ 2                 → SEQ_DONE 2  — final place
    11. BASE:STOP                            — stop system

  Every GOTO / ROTATE / SEQ step is timeout-protected (see _STEP_TIMEOUT_S
  and the per-step overrides in AUTO_SEQUENCE) — if the expected feedback
  doesn't arrive in time, the sequence aborts safely (BASE:STOP + ARM:STOP)
  rather than hanging forever.

Tuning constants (POS_MAX_SPEED, POS_AXIS_TOLERANCE_DEFAULT, etc.) live on
the ESP32 firmware, not here — this node only issues GOTO targets and reads
back POS:REACHED / SEQ_DONE.
"""

import queue
import time
import logging
import sys
import os
from enum import Enum, auto

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pi_robot.core.base_node      import BaseNode
from pi_robot.core.message_bus    import Topics
from pi_robot.core.pid_controller import PIDController
from config.config_loader import get as cfg

logger = logging.getLogger("decision_node")


class State(Enum):
    IDLE      = auto()   # Waiting
    MANUAL    = auto()   # Manual control (keyboard/PS4)
    PI_SEQ    = auto()   # Pi-side step-by-step arm sequence (move/delay/grip)

    # ── AUTO states (odometry-driven, per autonomous spec) ───────────────────
    AUTO_RESET        = auto()  # Step 0  — BASE:POS:RESET
    AUTO_GOTO_1       = auto()  # Step 1  — BASE:GOTO:0.50,0.00
    AUTO_ROTATE_CCW   = auto()  # Step 2  — rotate CCW until heading >= +90
    AUTO_GOTO_2       = auto()  # Step 3  — BASE:GOTO:0.50,0.50
    AUTO_PICK         = auto()  # Step 4  — SEQ 0 (pick RED cube)
    AUTO_GOTO_3       = auto()  # Step 5  — BASE:GOTO:0.50,1.00
    AUTO_PLACE_1      = auto()  # Step 6  — SEQ 1 (place zone 1)
    AUTO_ROTATE_CW    = auto()  # Step 7  — rotate CW until heading <= -180 (wrapped)
    AUTO_GOTO_4       = auto()  # Step 8  — BASE:GOTO:0.00,1.00
    AUTO_GOTO_5       = auto()  # Step 9  — BASE:GOTO:-0.30,0.70
    AUTO_PLACE_2      = auto()  # Step 10 — SEQ 2 (final place)
    AUTO_STOP         = auto()  # Step 11 — BASE:STOP
    AUTO_DONE         = auto()  # Sequence complete, idle until mode switch
    AUTO_ABORTED      = auto()  # A step timed out or errored — system stopped safely


# ── AUTO sequence step table ──────────────────────────────────────────────────
# Each entry describes how to *enter* a state (the command(s) to issue) and
# which State to advance to. GOTO / PICK / PLACE steps wait for feedback
# (POS_REACHED / SEQ_DONE); ROTATE steps poll heading via BASE:POS?.
# "timeout_s" overrides the default per-step timeout (see _STEP_TIMEOUT_S).
AUTO_GOTO_TARGETS = {
    State.AUTO_GOTO_1: (0.50,  0.00),
    State.AUTO_GOTO_2: (0.50,  0.50),
    State.AUTO_GOTO_3: (0.50,  1.00),
    State.AUTO_GOTO_4: (0.00,  1.00),
    State.AUTO_GOTO_5: (-0.30, 0.70),
}

AUTO_SEQ_BOX = {
    State.AUTO_PICK:    0,   # pick RED cube
    State.AUTO_PLACE_1: 1,   # place — zone 1
    State.AUTO_PLACE_2: 2,   # place — zone 2 (final)
}

# Linear progression through the spec's 11 steps.
AUTO_NEXT_STATE = {
    State.AUTO_RESET:      State.AUTO_GOTO_1,
    State.AUTO_GOTO_1:     State.AUTO_ROTATE_CCW,
    State.AUTO_ROTATE_CCW: State.AUTO_GOTO_2,
    State.AUTO_GOTO_2:     State.AUTO_PICK,
    State.AUTO_PICK:       State.AUTO_GOTO_3,
    State.AUTO_GOTO_3:     State.AUTO_PLACE_1,
    State.AUTO_PLACE_1:    State.AUTO_ROTATE_CW,
    State.AUTO_ROTATE_CW:  State.AUTO_GOTO_4,
    State.AUTO_GOTO_4:     State.AUTO_GOTO_5,
    State.AUTO_GOTO_5:     State.AUTO_PLACE_2,
    State.AUTO_PLACE_2:    State.AUTO_STOP,
    State.AUTO_STOP:       State.AUTO_DONE,
}

# Heading targets for the two rotation steps (degrees, relative to the
# heading at the moment the rotation step starts — the firmware wraps
# `heading` to [-180, 180), so we track *accumulated rotation* rather than
# trusting the raw wrapped value directly).
ROTATE_CCW_TARGET_DEG = 90.0    # Step 2:  stop once we've turned +90° CCW
ROTATE_CW_TARGET_DEG  = -180.0  # Step 7:  stop once we've turned -180° (CW)


# ── Pi-side autonomous arm sequence ──────────────────────────────────────────
# Runs step-by-step on the Raspberry Pi (not the ESP32). Each step is a dict:
#   {"type": "move",  "joint": 0-3, "deg": <float>}   -> ARM:JOINT j deg
#   {"type": "grip",  "deg": <0-180>}                  -> gripper to deg
#   {"type": "delay", "seconds": <float>}               -> wait
#   {"type": "home"}                                    -> ARM:HOME
# This is independent from the ESP32 hardware sequence triggered by R1
# (ARM:PICK / "SEQ 1"), which is left completely untouched.
DEFAULT_PI_SEQUENCE = [
    {"type": "home"},
    {"type": "delay", "seconds": 0.6},
    {"type": "grip",  "deg": 165},                  # open
    {"type": "move",  "joint": 0, "deg": -12.0},     # base toward box
    {"type": "delay", "seconds": 0.5},
    {"type": "move",  "joint": 1, "deg": 5.0},       # shoulder lower
    {"type": "move",  "joint": 2, "deg": 44.0},      # elbow approach
    {"type": "delay", "seconds": 0.5},
    {"type": "grip",  "deg": 105},                   # close on object
    {"type": "delay", "seconds": 0.4},
    {"type": "move",  "joint": 1, "deg": -15.0},     # shoulder raise
    {"type": "move",  "joint": 2, "deg": 34.0},      # elbow retract
    {"type": "delay", "seconds": 0.5},
    {"type": "move",  "joint": 0, "deg": 90.0},      # base to drop point
    {"type": "delay", "seconds": 0.5},
    {"type": "grip",  "deg": 165},                   # release
    {"type": "delay", "seconds": 0.4},
    {"type": "home"},
]


class DecisionNode(BaseNode):

    def __init__(self, **kwargs):
        super().__init__(name="decision_node", **kwargs)

        self._qr_inbox     = None
        self._qr_lost_box  = None
        self._feedback_box = None
        self._mode_box     = None
        self._manual_box   = None

        self._state      = State.MANUAL
        self._mode       = cfg("system", "mode", "MANUAL")

        # QR / color detection
        self._last_qr        = None
        self._lost_count     = 0
        self._cube_color     = "UNKNOWN"
        self._cube_color_rgb = [120, 120, 120]

        # ── AUTO sequence config ──────────────────────────────────────────────
        auto_cfg = cfg("robot", "auto_sequence", {})
        self._step_timeout_s   = auto_cfg.get("step_timeout_s",   15.0)
        self._rotate_timeout_s = auto_cfg.get("rotate_timeout_s", 8.0)
        self._seq_timeout_s    = auto_cfg.get("seq_timeout_s",    20.0)
        self._pos_poll_period  = auto_cfg.get("pos_poll_period_s", 0.1)

        # State timing / generic per-state bookkeeping
        self._state_start = None   # time.time() when current state was entered
        self._place_step  = 0      # retained for PI_SEQ bookkeeping only

        # ── Latest parsed feedback from serial_node (updated every loop) ─────
        self._latest_pos        = None   # {"x","y","heading","total_distance"}
        self._goto_reached      = False  # set True when POS_REACHED seen for current GOTO
        self._goto_reached_xy   = None
        self._seq_done_box      = None   # set to box number when SEQ_DONE <box> seen
        self._seq_aborted       = False
        self._last_pos_poll     = 0.0    # last time we sent BASE:POS? during a rotate step

        # Rotation tracking (accumulated, unwrapped) — needed because the
        # firmware reports heading wrapped to [-180, 180), so a 180° CW turn
        # can't be detected with a single threshold on the raw value alone.
        self._rotate_start_heading   = None
        self._rotate_accum_deg       = 0.0
        self._rotate_last_heading    = None

        # Pi-side step sequencer state
        self._pi_seq            = DEFAULT_PI_SEQUENCE
        self._pi_seq_idx        = 0
        self._pi_seq_step_start = None
        self._pi_seq_prev_state = State.MANUAL  # state to return to when done/aborted

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def setup(self):
        self._qr_inbox     = self.bus.subscribe(Topics.VISION_QR_DETECTED, maxsize=10)
        self._qr_lost_box  = self.bus.subscribe(Topics.VISION_QR_LOST,     maxsize=10)
        self._feedback_box = self.bus.subscribe(Topics.SERIAL_FEEDBACK,    maxsize=20)
        self._mode_box     = self.bus.subscribe(Topics.SYSTEM_MODE,        maxsize=5)
        self._manual_box   = self.bus.subscribe(Topics.MANUAL_COMMAND,     maxsize=50)

        self._state = State.MANUAL
        logger.info("decision_node ready — waiting for MANUAL control")
        logger.info("Press OPTIONS (PS4) or M (keyboard) to start AUTO sequence")

    def run(self):
        logger.info("decision_node running")
        while self.running:
            self._drain_mode()
            self._drain_feedback()
            self._drain_qr()

            if self._state == State.MANUAL:
                self._handle_manual()
            elif self._state == State.PI_SEQ:
                self._handle_manual(allow_abort_only=True)
                self._handle_pi_seq()
            else:
                self._handle_auto()

            time.sleep(0.05)

    # ── Mode switch ───────────────────────────────────────────────────────────

    def _drain_mode(self):
        while True:
            try:
                msg = self._mode_box.get_nowait()
                new_mode = msg["payload"].get("mode", "MANUAL").upper()
                logger.info(f"Mode switch → {new_mode}")

                if new_mode == "AUTO" and self._state == State.MANUAL:
                    logger.info("=" * 50)
                    logger.info("  AUTO SEQUENCE STARTING (odometry-driven)")
                    logger.info("=" * 50)
                    self._action("BASE:STOP")
                    self._reset_auto_tracking()
                    self._next(State.AUTO_RESET)

                elif new_mode == "MANUAL":
                    self._action("BASE:STOP")
                    self._action("ARM:HOME")
                    self._transition(State.MANUAL)

                self._mode = new_mode
            except queue.Empty:
                break

    def _drain_feedback(self):
        """
        Consume every queued SERIAL_FEEDBACK message and update the latest
        known pose / GOTO-arrival / SEQ-completion state.

        serial_node.py parses raw ESP32 lines into structured dicts under
        msg["payload"]["parsed"] — see serial_node._handle_feedback(). The
        AUTO state machine below reacts to these instead of guessing with
        fixed timers.
        """
        while True:
            try:
                msg = self._feedback_box.get_nowait()
            except queue.Empty:
                break

            parsed = msg["payload"].get("parsed", {}) or {}
            ptype  = parsed.get("type")

            if ptype == "POS":
                self._latest_pos = {
                    "x":       parsed.get("x"),
                    "y":       parsed.get("y"),
                    "heading": parsed.get("heading"),
                    "total_distance": parsed.get("total_distance"),
                }

            elif ptype == "POS_REACHED":
                self._goto_reached    = True
                self._goto_reached_xy = (parsed.get("x"), parsed.get("y"))

            elif ptype == "SEQ_DONE":
                self._seq_done_box = parsed.get("box")

            elif ptype == "SEQ_ABORTED":
                self._seq_aborted = True

            elif ptype == "ERR":
                logger.warning(f"ESP32 ERR feedback: {parsed.get('msg')!r}")

    def _drain_qr(self):
        # Keep latest QR detection
        latest = None
        while True:
            try:
                latest = self._qr_inbox.get_nowait()
            except queue.Empty:
                break

        if latest:
            self._last_qr    = latest["payload"]
            self._lost_count = 0
            # Parse color from QR text
            qr_text = self._last_qr.get("qr_data", "").lower()
            if "red"   in qr_text: self._cube_color = "RED"
            elif "green" in qr_text: self._cube_color = "GREEN"
            elif "blue"  in qr_text: self._cube_color = "BLUE"
            else:
                c = self._last_qr.get("color", "UNKNOWN")
                if c != "UNKNOWN": self._cube_color = c

        while True:
            try:
                self._qr_lost_box.get_nowait()
                self._lost_count += 1
            except queue.Empty:
                break

    # ── MANUAL mode ───────────────────────────────────────────────────────────

    def _handle_manual(self, allow_abort_only=False):
        """Pass all manual commands through directly.

        Previously this drained the whole queue and kept only the LAST
        message, silently discarding every other queued command. That
        dropped fast button presses (e.g. several quick D-pad taps) and
        any command that arrived just before a HOME/PICK press in the same
        50ms tick. Now every queued command is forwarded, in order.

        When allow_abort_only=True (used while a PI_SEQ is running), normal
        manual commands are ignored so they don't fight with the running
        sequence — only ARM:PI_SEQ_STOP / Share-emergency-stop (BASE:STOP)
        are honored, to abort the sequence early.
        """
        processed = 0
        while True:
            try:
                msg = self._manual_box.get_nowait()
            except queue.Empty:
                break
            cmd = msg["payload"].get("command", "")
            if not cmd:
                continue
            processed += 1

            cu = cmd.strip().upper()

            if cu == "ARM:PI_SEQ":
                if self._state != State.PI_SEQ:
                    self._start_pi_seq()
                continue

            if cu == "ARM:PI_SEQ_STOP":
                if self._state == State.PI_SEQ:
                    self._abort_pi_seq()
                continue

            if allow_abort_only:
                # Sequence is running — only abort-style commands apply.
                if cu == "BASE:STOP":
                    self._abort_pi_seq()
                continue

            logger.debug(f"Manual: {cmd}")
            self._action(cmd, source="manual")

        if processed == 0:
            time.sleep(0.02)

    # ── PI-SIDE STEP SEQUENCER (move / delay / grip) ────────────────────────────

    def _start_pi_seq(self):
        logger.info("=" * 50)
        logger.info("  PI-SIDE ARM SEQUENCE STARTING (%d steps)", len(self._pi_seq))
        logger.info("=" * 50)
        self._pi_seq_prev_state = self._state if self._state != State.PI_SEQ else State.MANUAL
        self._pi_seq_idx        = 0
        self._pi_seq_step_start = None
        self._transition(State.PI_SEQ)

    def _abort_pi_seq(self):
        logger.warning("PI_SEQ aborted by user")
        self._action("ARM:STOP")
        self._transition(self._pi_seq_prev_state)
        self._pi_seq_idx        = 0
        self._pi_seq_step_start = None

    def _handle_pi_seq(self):
        """Run the Pi-side sequence one step at a time.

        Step types:
          move  -> JOINT <joint> <deg>
          grip  -> GRIPPER <deg>
          delay -> wait <seconds>
          home  -> HOME
        Each step (except delay) is sent once, then we wait a fixed small
        settle time before advancing — this runs entirely on the Pi side,
        independent of the ESP32 hardware SEQ used by R1.
        """
        now = time.time()

        if self._pi_seq_idx >= len(self._pi_seq):
            logger.info("PI_SEQ complete — returning to %s", self._pi_seq_prev_state.name)
            self._transition(self._pi_seq_prev_state)
            self._pi_seq_idx        = 0
            self._pi_seq_step_start = None
            return

        step = self._pi_seq[self._pi_seq_idx]

        if self._pi_seq_step_start is None:
            # Entering a new step — issue the command once.
            self._pi_seq_step_start = now
            kind = step.get("type")
            if kind == "move":
                self._action("ARM:RAW:JOINT %d %.1f" % (step["joint"], step["deg"]))
            elif kind == "grip":
                self._action("ARM:RAW:GRIPPER %d" % int(step["deg"]))
            elif kind == "home":
                self._action("ARM:HOME")
            elif kind == "delay":
                pass  # nothing to send, just wait
            else:
                logger.warning(f"PI_SEQ unknown step type: {kind!r} — skipping")
                self._pi_seq_idx += 1
                self._pi_seq_step_start = None
                return
            logger.info(f"PI_SEQ step {self._pi_seq_idx + 1}/{len(self._pi_seq)}: {step}")

        # Determine wait time for this step, then advance.
        wait_s = step.get("seconds", 0.5) if step.get("type") == "delay" else 0.5
        if now - self._pi_seq_step_start >= wait_s:
            self._pi_seq_idx += 1
            self._pi_seq_step_start = None

    # ── AUTO mode (odometry-driven, per spec) ────────────────────────────────

    def _reset_auto_tracking(self):
        """Clear all per-run AUTO tracking state. Call once when AUTO starts."""
        self._latest_pos      = None
        self._goto_reached    = False
        self._goto_reached_xy = None
        self._seq_done_box    = None
        self._seq_aborted     = False
        self._last_pos_poll   = 0.0
        self._rotate_start_heading = None
        self._rotate_accum_deg     = 0.0
        self._rotate_last_heading  = None

    def _handle_auto(self):
        """
        Drive the 11-step odometry-based autonomous sequence.

        Each AUTO_* state is entered once (commands issued), then this method
        is called every loop tick to check whether the expected feedback
        (POS_REACHED / SEQ_DONE / heading threshold) has arrived, or whether
        the step has timed out.
        """
        now = time.time()
        entering = self._state_start is None
        if entering:
            self._state_start = now
            logger.info(f"AUTO: entering {self._state.name}")
            self._enter_auto_state(self._state)

        elapsed = now - self._state_start

        if self._state in AUTO_GOTO_TARGETS:
            self._step_goto(elapsed)
        elif self._state == State.AUTO_RESET:
            self._step_reset(elapsed)
        elif self._state in (State.AUTO_ROTATE_CCW, State.AUTO_ROTATE_CW):
            self._step_rotate(now, elapsed)
        elif self._state in AUTO_SEQ_BOX:
            self._step_seq(elapsed)
        elif self._state == State.AUTO_STOP:
            self._step_stop()
        elif self._state == State.AUTO_DONE:
            pass  # terminal — stay here until operator switches back to MANUAL
        elif self._state == State.AUTO_ABORTED:
            pass  # terminal — system already stopped safely

    # ── Per-state entry actions ──────────────────────────────────────────────

    def _enter_auto_state(self, state):
        """Issue the command(s) for the step we just entered."""
        if state == State.AUTO_RESET:
            self._action("BASE:POS:RESET")

        elif state in AUTO_GOTO_TARGETS:
            x, y = AUTO_GOTO_TARGETS[state]
            self._goto_reached    = False
            self._goto_reached_xy = None
            self._action("BASE:GOTO", x=x, y=y)

        elif state == State.AUTO_ROTATE_CCW:
            self._rotate_start_heading = None  # set on first POS feedback
            self._rotate_accum_deg     = 0.0
            self._rotate_last_heading  = None
            self._last_pos_poll        = 0.0
            self._action("BASE:ROTATECCW")

        elif state == State.AUTO_ROTATE_CW:
            self._rotate_start_heading = None
            self._rotate_accum_deg     = 0.0
            self._rotate_last_heading  = None
            self._last_pos_poll        = 0.0
            self._action("BASE:ROTATECW")

        elif state in AUTO_SEQ_BOX:
            self._seq_done_box = None
            self._seq_aborted  = False
            box = AUTO_SEQ_BOX[state]
            self._action(f"ARM:RUNSEQ:{box}")

        elif state == State.AUTO_STOP:
            self._action("BASE:STOP")

    # ── Per-state step checks ────────────────────────────────────────────────

    def _step_reset(self, elapsed):
        """Step 0 — wait briefly for OK:POS_RESET-equivalent settle, then advance.

        The firmware doesn't gate anything on POS:RESET besides zeroing
        odometry, so a short fixed settle (rather than a feedback wait) is
        sufficient and keeps this step simple and bounded.
        """
        if elapsed >= 0.3:
            self._advance_auto()

    def _step_goto(self, elapsed):
        """Steps 1, 3, 5, 8, 9 — wait for POS:REACHED, with timeout."""
        if self._goto_reached:
            logger.info(
                f"AUTO: {self._state.name} reached "
                f"({self._goto_reached_xy[0]:.3f}, {self._goto_reached_xy[1]:.3f})"
            )
            self._advance_auto()
            return

        if elapsed > self._step_timeout_s:
            self._abort_auto(f"{self._state.name} timed out waiting for POS:REACHED "
                              f"after {self._step_timeout_s:.1f}s")

    def _step_rotate(self, now, elapsed):
        """
        Steps 2, 7 — rotate until accumulated heading change reaches target.

        We poll BASE:POS? periodically and track *accumulated* rotation
        (handling the wraparound at ±180°) rather than comparing the raw
        wrapped heading to a fixed threshold, since a CW turn that crosses
        the -180/+180 boundary can't be detected with a single comparison
        on the wrapped value alone.
        """
        if now - self._last_pos_poll >= self._pos_poll_period:
            self._action("BASE:POS?")
            self._last_pos_poll = now

        heading = self._latest_pos.get("heading") if self._latest_pos else None
        if heading is not None:
            if self._rotate_start_heading is None:
                self._rotate_start_heading = heading
                self._rotate_last_heading  = heading
            else:
                delta = heading - self._rotate_last_heading
                # Unwrap a single step's delta across the ±180 boundary.
                if delta > 180.0:
                    delta -= 360.0
                elif delta < -180.0:
                    delta += 360.0
                self._rotate_accum_deg += delta
                self._rotate_last_heading = heading

        target = (ROTATE_CCW_TARGET_DEG if self._state == State.AUTO_ROTATE_CCW
                  else ROTATE_CW_TARGET_DEG)

        reached = (self._rotate_accum_deg >= target if target > 0
                   else self._rotate_accum_deg <= target)

        if reached:
            self._action("BASE:STOP")
            logger.info(
                f"AUTO: {self._state.name} reached target "
                f"({self._rotate_accum_deg:.1f}° accumulated)"
            )
            self._advance_auto()
            return

        if elapsed > self._rotate_timeout_s:
            self._action("BASE:STOP")
            self._abort_auto(f"{self._state.name} timed out after "
                              f"{self._rotate_timeout_s:.1f}s "
                              f"(reached {self._rotate_accum_deg:.1f}° of {target:.1f}°)")

    def _step_seq(self, elapsed):
        """Steps 4, 6, 10 — wait for SEQ_DONE <box>, with timeout/abort handling."""
        expected_box = AUTO_SEQ_BOX[self._state]

        if self._seq_aborted:
            self._abort_auto(f"{self._state.name}: ESP32 reported SEQ_ABORTED")
            return

        if self._seq_done_box is not None:
            if self._seq_done_box == expected_box:
                logger.info(f"AUTO: {self._state.name} — SEQ_DONE {expected_box}")
                self._advance_auto()
            else:
                logger.warning(
                    f"AUTO: {self._state.name} expected SEQ_DONE {expected_box}, "
                    f"got {self._seq_done_box} — proceeding anyway"
                )
                self._advance_auto()
            return

        if elapsed > self._seq_timeout_s:
            self._abort_auto(f"{self._state.name} timed out waiting for "
                              f"SEQ_DONE {expected_box} after {self._seq_timeout_s:.1f}s")

    def _step_stop(self):
        """Step 11 — final BASE:STOP already issued on entry; advance immediately."""
        self._advance_auto()

    # ── Sequence flow control ────────────────────────────────────────────────

    def _advance_auto(self):
        """Move to the next state in AUTO_NEXT_STATE, or finish if at the end."""
        nxt = AUTO_NEXT_STATE.get(self._state)
        if nxt is None:
            logger.info("=" * 50)
            logger.info("  AUTO SEQUENCE COMPLETE!")
            logger.info("=" * 50)
            self._next(State.AUTO_DONE)
        else:
            self._next(nxt)

    def _abort_auto(self, reason):
        """Stop everything safely and park in AUTO_ABORTED with a logged reason."""
        logger.error(f"AUTO ABORTED: {reason}")
        self._action("BASE:STOP")
        self._action("ARM:STOP")
        self._next(State.AUTO_ABORTED)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _next(self, new_state):
        """Transition to next state and reset timer."""
        self._transition(new_state)
        self._state_start = None
        self._place_step  = 0

    def _action(self, command: str, **params):
        self.publish(
            Topics.DECISION_ACTION,
            command = command,
            params  = params,
            state   = self._state.name,
            mode    = self._mode,
            color   = self._cube_color,
        )

    def _transition(self, new_state):
        if new_state != self._state:
            logger.info(f"STATE: {self._state.name} → {new_state.name}")
            self._state = new_state

    def _drain_latest(self, q):
        latest = None
        while True:
            try: latest = q.get_nowait()
            except queue.Empty: break
        return latest
