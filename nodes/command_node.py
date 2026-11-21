"""
command_node.py
---------------
Translates high-level actions into exact serial strings
for the two ESP32 firmwares.

BASE (esp_base.ino) — UART USB /dev/ttyACM0:
  BASE:FORWARD / BACKWARD / LEFT / RIGHT
  BASE:ROTATECW / ROTATECCW / STOP
  BASE:SPEED:<0-255>
  BASE:PID:<kp>,<ki>,<kd>

ARM (robot_arm_merged.ino) — Serial2 GPIO16/17 /dev/ttyUSB0:
  TYPE A compact frame (PS4 joystick, no reply):
    $<j0>,<j1>,<j2>,<j3>,<grip>\n
  TYPE B ASCII commands:
    HOME / STOP / GRIPPER <deg> / JOINT <j> <deg>
    SPEED <ms> / RUNSEQ [n] / STOPSEQ / SEQSTATE
"""

import queue, re, logging, sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from pi_robot.core.base_node   import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("command_node")


def _pct_to_esp(pct):
    return int(max(0, min(255, pct * 255 / 100)))


class CommandNode(BaseNode):

    def __init__(self, **kwargs):
        super().__init__(name="command_node", **kwargs)
        self._inbox       = None
        self._default_spd = cfg("robot", "base", {}).get("default_speed", 40)
        # Track arm joint state for compact frame
        self._arm_j = [0.0, 0.0, 0.0, 0.0]
        self._arm_g = 90

    def setup(self):
        self._inbox = self.bus.subscribe(Topics.DECISION_ACTION, maxsize=50)
        init_spd = _pct_to_esp(self._default_spd)
        self.publish(Topics.COMMAND_SERIAL,
                     command_str=f"BASE:SPEED:{init_spd}\n")
        logger.info(f"command_node ready — default speed {self._default_spd}% → ESP {init_spd}")

    def run(self):
        logger.info("command_node running")
        while self.running:
            try:
                msg = self._inbox.get(timeout=1.0)
            except queue.Empty:
                continue

            payload = msg["payload"]
            command = payload.get("command", "")
            params  = payload.get("params",  {})

            cmd_str = self._build(command, params)
            if cmd_str:
                logger.debug(f"CMD: {repr(cmd_str.strip())}")
                self.publish(Topics.COMMAND_SERIAL, command_str=cmd_str)
            elif command:
                logger.warning(f"Unknown action: '{command}'")

    def _build(self, action, params):
        a = action.strip()
        au = a.upper()

        # ── ARM compact frame passthrough (PS4 joystick) ─────────────────────
        # Format: $j0,j1,j2,j3,grip\n
        if au.startswith("ARM:COMPACT:"):
            return a[12:] + "\n"   # strip prefix, send raw frame

        # ── ARM ASCII passthrough ─────────────────────────────────────────────
        if au.startswith("ARM:RAW:"):
            raw = a[8:]   # strip ARM:RAW:
            return f"{raw}\n"

        if au.startswith("ARM:SPEED:"):
            ms = a[10:]
            return f"SPEED {ms}\n"

        # ── ARM high-level → firmware commands ───────────────────────────────
        #
        # SEQ box mapping (per autonomous spec):
        #   SEQ 0 → pick RED cube
        #   SEQ 1 → place at location 1
        #   SEQ 2 → place at location 2
        arm_map = {
            "ARM:HOME":     "HOME\n",
            "ARM:STOP":     "STOP\n",
            "ARM:GRIP":     "GRIPPER 0\n",
            "ARM:PLACE":    "GRIPPER 180\n",
            "ARM:LOWER":    "JOINT 2 -69.0\n",
            "ARM:RAISE":    "JOINT 2 -30.0\n",
            "ARM:PICK":     "SEQ 0\n",       # pick RED cube
            "ARM:PLACE1":   "SEQ 1\n",       # place — location 1
            "ARM:PLACE2":   "SEQ 2\n",       # place — location 2
            "ARM:RUNSEQ":   "SEQ 0\n",       # legacy alias — now matches SEQ 0 = pick
            "ARM:STOPSEQ":  "SEQ_STOP\n",
            "ARM:SEQSTATE": "SEQ_STATE\n",
        }
        if au in arm_map:
            return arm_map[au]

        # ── ARM:RUNSEQ:n — explicit box selection (0/1/2) ─────────────────────
        m_seq = re.match(r"^ARM:RUNSEQ:(\d)$", au)
        if m_seq:
            return f"SEQ {m_seq.group(1)}\n"

        # ── BASE movement ─────────────────────────────────────────────────────
        if au == "BASE:FORWARD":
            spd = _pct_to_esp(int(params.get("speed", self._default_spd)))
            return f"BASE:SPEED:{spd}\nBASE:FORWARD\n"

        if au == "BASE:BACKWARD":
            spd = _pct_to_esp(int(params.get("speed", self._default_spd)))
            return f"BASE:SPEED:{spd}\nBASE:BACKWARD\n"

        if au == "BASE:TURN":
            # Generic turn action — direction comes from params, defaults RIGHT/CW.
            direction = str(params.get("direction", "RIGHT")).upper()
            spd = _pct_to_esp(int(params.get("speed",
                              cfg("robot","base",{}).get("turn_speed",35))))
            cmd = "BASE:ROTATECW" if direction in ("RIGHT","CW") else "BASE:ROTATECCW"
            return f"BASE:SPEED:{spd}\n{cmd}\n"

        if au in ("BASE:ROTATECW", "BASE:ROTATECCW"):
            # Explicit direction in the command itself — params only override speed.
            # (Previously this branch was merged with BASE:TURN and always fell
            # back to the "RIGHT"/CW default, so BASE:ROTATECCW silently rotated
            # CW. Keep direction tied to the command name, not a params guess.)
            spd = _pct_to_esp(int(params.get("speed",
                              cfg("robot","base",{}).get("turn_speed",35))))
            return f"BASE:SPEED:{spd}\n{au}\n"

        if au in ("BASE:STRAFE","BASE:LEFT"):
            spd = _pct_to_esp(int(params.get("speed", self._default_spd)))
            return f"BASE:SPEED:{spd}\nBASE:LEFT\n"

        if au == "BASE:RIGHT":
            spd = _pct_to_esp(int(params.get("speed", self._default_spd)))
            return f"BASE:SPEED:{spd}\nBASE:RIGHT\n"

        if au == "BASE:STOP":
            return "BASE:STOP\n"

        if au == "BASE:SPEED":
            spd = _pct_to_esp(int(params.get("speed", self._default_spd)))
            return f"BASE:SPEED:{spd}\n"

        if au == "BASE:PID":
            kp = params.get("kp", 4.0)
            ki = params.get("ki", 0.03)
            kd = params.get("kd", 0.0)
            return f"BASE:PID:{kp},{ki},{kd}\n"

        # ── BASE position controller (odometry-based GOTO) ───────────────────
        if au == "BASE:GOTO":
            # params: x, y, tolerance (optional)
            x = params.get("x", 0.0)
            y = params.get("y", 0.0)
            tol = params.get("tolerance")
            if tol is not None:
                return f"BASE:GOTO:{x:.4f},{y:.4f},{tol:.4f}\n"
            return f"BASE:GOTO:{x:.4f},{y:.4f}\n"

        if au == "BASE:GOTO:CANCEL":
            return "BASE:GOTO:CANCEL\n"

        if au == "BASE:POS?":
            return "BASE:POS?\n"

        if au == "BASE:POS:RESET":
            return "BASE:POS:RESET\n"

        if au.startswith("BASE:"):
            return f"{a}\n"

        return ""
