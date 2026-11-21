#!/usr/bin/env python3
"""
serial_node.py — UART bridge for BASE ESP32-S3 and ARM ESP32

ASCII protocol matching the ESP32 firmware (esp_base176.ino, Arduino core 3.3.8).

BASE ESP32-S3 Protocol (ASCII, 115200 baud):
  TX (Pi → BASE):
    BASE:FORWARD\n
    BASE:BACKWARD\n
    BASE:LEFT\n
    BASE:RIGHT\n
    BASE:ROTATECW\n
    BASE:ROTATECCW\n
    BASE:STOP\n
    BASE:SPEED:<0-255>\n      (PWM duty / global speed)
    BASE:PID:<kp>,<ki>,<kd>\n  (yaw PID tuning)

  RX (BASE → Pi):
    OK:<cmd>\n
    ODOM:<fl>,<fr>,<rl>,<rr>\n
    IMU:<yaw_deg>\n
    ERR:<msg>\n

ARM ESP32 Protocol (generic passthrough — adjust when ARM firmware is known):
  TX: HOME, JOINT j deg, IK x y z, GRIPPER deg, POSE q1 q2 q3 q4, ...
  RX: PONG, OK cmd, STATE ..., ANGLES:..., SEQSTATE ...
"""

import queue
import re
import threading
import time
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from core.base_node import BaseNode
from core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("serial_node")

# ── Serial port helper ─────────────────────────────────────
class SerialPort:
    """Thread-safe serial port with auto-reconnect."""

    def __init__(self, name, port, baud, timeout=1.0, retries=3, retry_delay=2.0):
        self.name = name
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.retries = retries
        self.retry_delay = retry_delay
        self._ser = None
        self._lock = threading.Lock()
        self._sim = False

    def connect(self):
        if not SERIAL_AVAILABLE:
            logger.warning(f"[{self.name}] pyserial not installed — SIM mode")
            self._sim = True
            return False

        if self._ser and self._ser.is_open:
            self._sim = False
            return True

        for attempt in range(1, self.retries + 1):
            try:
                self._ser = serial.Serial(
                    port=self.port, baudrate=self.baud,
                    timeout=self.timeout)
                time.sleep(0.2)
                self._ser.reset_input_buffer()
                logger.info(f"[{self.name}] Connected: {self.port} @ {self.baud}")
                self._sim = False
                return True
            except Exception as e:
                logger.warning(f"[{self.name}] Attempt {attempt}/{self.retries}: {e}")
                if attempt < self.retries:
                    time.sleep(min(self.retry_delay, 0.5))

        logger.error(f"[{self.name}] Cannot open {self.port} — SIM mode")
        self._sim = True
        return False

    def send(self, cmd):
        """Send ASCII command with newline."""
        line = cmd if cmd.endswith("\n") else cmd + "\n"
        with self._lock:
            try:
                if self._sim or not self._ser or not self._ser.is_open:
                    if not self.connect():
                        logger.info(f"[SIM:{self.name}] TX: {repr(cmd.strip())}")
                        return
                self._ser.write(line.encode("utf-8"))
                self._ser.flush()
                logger.debug(f"[{self.name}] TX: {repr(cmd.strip())}")
            except Exception as e:
                logger.error(f"[{self.name}] TX error: {e} — trying reconnect")
                self._sim = True
                try:
                    self.connect()
                except Exception:
                    pass

    def readline(self):
        if self._sim or not self._ser or not self._ser.is_open:
            try:
                if self.connect():
                    logger.debug(f"[{self.name}] reconnected successfully")
                else:
                    return ""
            except Exception:
                return ""
        try:
            line = self._ser.readline()
            if line:
                return line.decode("utf-8", errors="replace").strip()
        except Exception as e:
            logger.error(f"[{self.name}] RX error: {e}")
            self._sim = True
            try:
                self.connect()
            except Exception:
                pass
        return ""

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info(f"[{self.name}] Closed {self.port}")

    @property
    def is_sim(self):
        return self._sim

# ── Serial Node ────────────────────────────────────────────
class SerialNode(BaseNode):
    def __init__(self, **kwargs):
        super().__init__(name="serial_node", **kwargs)
        self._inbox = None
        self._base_ser = None
        self._arm_ser = None

    # ── Lifecycle ──────────────────────────────────────────
    def setup(self):
        self._inbox = self.bus.subscribe(Topics.COMMAND_SERIAL, maxsize=50)

        base_cfg = cfg("serial", "base", {})
        self._base_ser = SerialPort(
            name="BASE",
            port=base_cfg.get("port", "/dev/ttyACM0"),
            baud=base_cfg.get("baud_rate", 115200),
            timeout=base_cfg.get("timeout", 1.0),
            retries=base_cfg.get("retry_attempts", 3),
            retry_delay=base_cfg.get("retry_delay", 2.0),
        )
        self._base_ser.connect()

        arm_cfg = cfg("serial", "arm", {})
        self._arm_ser = SerialPort(
            name="ARM",
            port=arm_cfg.get("port", "/dev/ttyUSB0"),
            baud=arm_cfg.get("baud_rate", 115200),
            timeout=arm_cfg.get("timeout", 1.0),
            retries=arm_cfg.get("retry_attempts", 3),
            retry_delay=arm_cfg.get("retry_delay", 2.0),
        )
        self._arm_ser.connect()

        # Start feedback reader threads
        for ser in (self._base_ser, self._arm_ser):
            threading.Thread(
                target=self._reader_loop, args=(ser,),
                name=f"reader_{ser.name}", daemon=True
            ).start()

        logger.info(
            f"serial_node ready — "
            f"BASE={self._base_ser.port}(sim={self._base_ser.is_sim}) "
            f"ARM={self._arm_ser.port}(sim={self._arm_ser.is_sim})"
        )

    def run(self):
        logger.info("serial_node running")

        while self.running:
            processed = 0
            while True:
                try:
                    msg = self._inbox.get_nowait()
                except queue.Empty:
                    break

                cmd_str = msg["payload"].get("command_str", "").strip()
                if not cmd_str:
                    continue

                for line in cmd_str.split("\n"):
                    line = line.strip()
                    if line:
                        self._route(line)
                        processed += 1

            if processed == 0:
                time.sleep(0.02)

    def cleanup(self):
        try:
            self._base_ser.send("BASE:STOP")
        except Exception:
            pass
        if self._base_ser:
            self._base_ser.close()
        if self._arm_ser:
            self._arm_ser.close()

    # ── Routing ────────────────────────────────────────────
    def _route(self, cmd_str):
        """Route one command line to the correct ESP32."""
        upper = cmd_str.upper().strip()

        # Compact PS4 joystick frame: $j0,j1,j2,j3,grip
        if cmd_str.startswith("$"):
            self._handle_compact_arm(cmd_str)
            return

        # BASE commands
        if upper.startswith("BASE:"):
            self._route_base(upper, cmd_str)
            return

        # ARM commands (prefixed)
        if upper.startswith("ARM:"):
            self._route_arm(upper, cmd_str)
            return

        # Raw ARM firmware commands (no prefix) that the command node emits.
        if re.match(r"^(HOME|STATE|PING|SEQ_STATE|SEQ_STOP|SEQ\s+\d+|JOINT\s+\d+\s+[-+]?\d*\.?\d+|GRIPPER\s+[-+]?\d*\.?\d+|POSE\s+[-+]?\d*\.?\d+(\s+[-+]?\d*\.?\d+){3}|IK\s+[-+]?\d*\.?\d+(\s+[-+]?\d*\.?\d+){2}|SPEED\s+\d+)$", upper):
            self._arm_ser.send(cmd_str.strip())
            return

        # Fallback: non-prefixed STOP goes to BASE as BASE:STOP
        if upper == "STOP":
            self._base_ser.send("BASE:STOP")
            return

        logger.warning(f"Unknown command: {repr(cmd_str)}")

    def _route_base(self, upper, original):
        """Route BASE:* commands to BASE ESP32.

        The ESP32 firmware expects the exact ASCII commands below.
        """
        # BASE:SPEED:<val>  (ESP32 expects 0-255 PWM duty)
        m = re.match(r"^BASE:SPEED:(\d+)$", upper)
        if m:
            self._base_ser.send(f"BASE:SPEED:{m.group(1)}")
            return

        # BASE:PID:<kp>,<ki>,<kd>
        m = re.match(r"^BASE:PID:(.+)$", upper)
        if m:
            self._base_ser.send(f"BASE:PID:{m.group(1)}")
            return

        # BASE:GOTO:<x>,<y>  or  BASE:GOTO:<x>,<y>,<tolerance>  or  BASE:GOTO:CANCEL
        # Numeric args may be negative/decimal, so match against the original
        # (mixed-case-safe) string rather than the upper-cased one — the
        # numbers themselves don't change case, but this keeps the regex
        # simple and matches the firmware's sscanf("%f,%f,%f") parsing.
        if upper.startswith("BASE:GOTO:"):
            args = original[len("BASE:GOTO:"):]
            if upper[len("BASE:GOTO:"):] == "CANCEL":
                self._base_ser.send("BASE:GOTO:CANCEL")
            elif re.match(r"^-?\d+(\.\d+)?,-?\d+(\.\d+)?(,-?\d+(\.\d+)?)?$", args):
                self._base_ser.send(f"BASE:GOTO:{args}")
            else:
                logger.warning(f"Malformed BASE:GOTO args: {repr(original)}")
            return

        # BASE:POS?  — request current odometry pose
        if upper == "BASE:POS?":
            self._base_ser.send("BASE:POS?")
            return

        # BASE:POS:RESET — zero odometry
        if upper == "BASE:POS:RESET":
            self._base_ser.send("BASE:POS:RESET")
            return

        # Direct motion commands (exact passthrough)
        passthrough = {
            "BASE:FORWARD":   "BASE:FORWARD",
            "BASE:BACKWARD":  "BASE:BACKWARD",
            "BASE:LEFT":      "BASE:LEFT",
            "BASE:RIGHT":     "BASE:RIGHT",
            "BASE:ROTATECW":  "BASE:ROTATECW",
            "BASE:ROTATECCW": "BASE:ROTATECCW",
            "BASE:STOP":      "BASE:STOP",
        }
        if upper in passthrough:
            self._base_ser.send(passthrough[upper])
            return

        logger.warning(f"Unknown BASE command: {repr(original)}")

    def _route_arm(self, upper, original):
        """Route ARM:* commands to ARM ESP32."""
        # ARM:RAW:<cmd> — raw passthrough
        if upper.startswith("ARM:RAW:"):
            raw = original[8:]  # strip "ARM:RAW:"
            self._arm_ser.send(raw)
            return

        # ARM:SPEED:<ms>
        m = re.match(r"^ARM:SPEED:(\d+)$", upper)
        if m:
            self._arm_ser.send(f"SPEED {m.group(1)}")
            return

        # ARM:JOINT:j:deg
        m = re.match(r"^ARM:JOINT:(\d+):([-\d.]+)$", upper)
        if m:
            self._arm_ser.send(f"JOINT {m.group(1)} {m.group(2)}")
            return

        # ARM:IK:x:y:z
        m = re.match(r"^ARM:IK:([-\d.]+):([-\d.]+):([-\d.]+)$", upper)
        if m:
            self._arm_ser.send(f"IK {m.group(1)} {m.group(2)} {m.group(3)}")
            return

        # ARM:GRIPPER:deg
        m = re.match(r"^ARM:GRIPPER:(\d+)$", upper)
        if m:
            self._arm_ser.send(f"GRIPPER {m.group(1)}")
            return

        # ARM:POSE:q1:q2:q3:q4
        m = re.match(r"^ARM:POSE:([-\d.]+):([-\d.]+):([-\d.]+):([-\d.]+)$", upper)
        if m:
            self._arm_ser.send(f"POSE {m.group(1)} {m.group(2)} {m.group(3)} {m.group(4)}")
            return

        # ARM:RUNSEQ or ARM:RUNSEQ:n  (new firmware protocol uses SEQ)
        m = re.match(r"^ARM:RUNSEQ(?::(\d+))?$", upper)
        if m:
            n = m.group(1) or "0"
            self._arm_ser.send(f"SEQ {n}")
            return

        # Simple passthrough
        passthrough = {
            "ARM:HOME":     "HOME",
            "ARM:STOP":     "STOP",
            "ARM:STATE":    "STATE",
            "ARM:STOPSEQ":  "SEQ_STOP",
            "ARM:SEQSTATE": "SEQ_STATE",
            "ARM:PING":     "PING",
        }
        if upper in passthrough:
            self._arm_ser.send(passthrough[upper])
            return

        # Generic ARM:CMD → strip prefix and send
        stripped = original[4:]  # remove "ARM:"
        logger.debug(f"ARM generic: {repr(stripped)}")
        self._arm_ser.send(stripped)

    def _handle_compact_arm(self, cmd_str):
        """
        Handle PS4 compact frame: $j0,j1,j2,j3,grip

        IMPORTANT: the ESP32 ARM firmware parses this exact "$j0,j1,j2,j3,grip"
        frame natively (see parseCompactFrame() in the .ino). It does NOT
        understand a "POSE" command. Previously this method rewrote the frame
        into "POSE ..." + "GRIPPER ..." which the firmware rejected with
        "ERR unknown command", silently dropping every joystick / D-pad /
        face-button arm movement. Fix: forward the frame to the ESP32 as-is.
        """
        try:
            parts = cmd_str[1:].split(",")  # strip "$"
            if len(parts) < 5:
                logger.debug(f"Compact frame malformed (too few parts): {cmd_str!r}")
                return
            j0, j1, j2, j3 = [float(p) for p in parts[:4]]
            grip = int(float(parts[4]))
            frame = f"${j0:.1f},{j1:.1f},{j2:.1f},{j3:.1f},{grip}"
            self._arm_ser.send(frame)
        except Exception as e:
            logger.debug(f"Compact frame error: {e} (raw={cmd_str!r})")

    # ── Feedback readers ───────────────────────────────────
    def _reader_loop(self, ser_port):
        """Read lines from ESP32 and publish to bus."""
        logger.info(f"Reader started: [{ser_port.name}]")
        while self.running:
            line = ser_port.readline()
            if line:
                logger.debug(f"[{ser_port.name}] RX: {line!r}")
                self._handle_feedback(line, source=ser_port.name.lower())
            else:
                time.sleep(0.01)

    def _handle_feedback(self, line, source):
        """Parse feedback and publish to Topics.SERIAL_FEEDBACK."""
        parsed = {"raw": line, "source": source}

        if source == "base":
            # Split only once: TYPE:payload
            parts = line.split(":", 1)
            msg_type = parts[0] if parts else ""
            payload = parts[1] if len(parts) > 1 else ""

            if msg_type == "ODOM":
                # ODOM:fl,fr,rl,rr
                try:
                    enc = [int(p) for p in payload.split(",")]
                    self.publish(
                        Topics.SERIAL_FEEDBACK,
                        raw=line,
                        parsed={"type": "ODOM", "enc": enc},
                        source=source,
                    )
                    return
                except Exception:
                    pass

            elif msg_type == "IMU":
                # IMU:yaw_deg
                try:
                    self.publish(
                        Topics.SERIAL_FEEDBACK,
                        raw=line,
                        parsed={"type": "IMU", "yaw": float(payload)},
                        source=source,
                    )
                    return
                except Exception:
                    pass

            elif msg_type == "POS":
                # Two distinct shapes from esp_base firmware:
                #   POS:REACHED:<x>,<y>                   (GOTO arrival)
                #   POS:<x>,<y>,<heading>,<totalDistance>  (POS? reply / pose stream)
                try:
                    if payload.startswith("REACHED:"):
                        x_str, y_str = payload[len("REACHED:"):].split(",")
                        self.publish(
                            Topics.SERIAL_FEEDBACK,
                            raw=line,
                            parsed={
                                "type": "POS_REACHED",
                                "x": float(x_str),
                                "y": float(y_str),
                            },
                            source=source,
                        )
                        return
                    else:
                        x_str, y_str, hd_str, dist_str = payload.split(",")
                        self.publish(
                            Topics.SERIAL_FEEDBACK,
                            raw=line,
                            parsed={
                                "type": "POS",
                                "x": float(x_str),
                                "y": float(y_str),
                                "heading": float(hd_str),
                                "total_distance": float(dist_str),
                            },
                            source=source,
                        )
                        return
                except Exception:
                    pass

            elif msg_type == "OK":
                self.publish(
                    Topics.SERIAL_FEEDBACK,
                    raw=line,
                    parsed={"type": "OK", "cmd": payload},
                    source=source,
                )
                return

            elif msg_type == "ERR":
                self.publish(
                    Topics.SERIAL_FEEDBACK,
                    raw=line,
                    parsed={"type": "ERR", "msg": payload},
                    source=source,
                )
                return

        elif source == "arm":
            # Space-separated ARM firmware lines, e.g.:
            #   SEQ_STEP 6 box=1 (Gripper close)
            #   SEQ_DONE 1
            #   SEQ_ABORTED
            #   SEQ_IDLE
            #   OK SEQ / ERR <reason>
            parts = line.split(" ", 1)
            msg_type = parts[0] if parts else ""
            rest = parts[1] if len(parts) > 1 else ""

            if msg_type == "SEQ_DONE":
                try:
                    self.publish(
                        Topics.SERIAL_FEEDBACK,
                        raw=line,
                        parsed={"type": "SEQ_DONE", "box": int(rest.strip())},
                        source=source,
                    )
                    return
                except Exception:
                    pass

            elif msg_type == "SEQ_ABORTED":
                self.publish(
                    Topics.SERIAL_FEEDBACK,
                    raw=line,
                    parsed={"type": "SEQ_ABORTED"},
                    source=source,
                )
                return

            elif msg_type == "SEQ_IDLE":
                self.publish(
                    Topics.SERIAL_FEEDBACK,
                    raw=line,
                    parsed={"type": "SEQ_IDLE"},
                    source=source,
                )
                return

            elif msg_type == "SEQ_STEP":
                # "6 box=1 (Gripper close)" or "6 box=1"
                try:
                    step_str, remainder = rest.split(" box=", 1)
                    box_str = remainder.split(" ", 1)[0]
                    self.publish(
                        Topics.SERIAL_FEEDBACK,
                        raw=line,
                        parsed={
                            "type": "SEQ_STEP",
                            "step": int(step_str.strip()),
                            "box": int(box_str.strip()),
                        },
                        source=source,
                    )
                    return
                except Exception:
                    pass

            parsed["type"] = msg_type
            parsed["data"] = rest

        # Publish raw/unparsed feedback
        self.publish(
            Topics.SERIAL_FEEDBACK,
            raw=line,
            parsed=parsed,
            source=source,
    )
