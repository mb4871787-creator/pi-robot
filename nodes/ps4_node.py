"""
ps4_node.py - PS4 DualShock 4 controller for pi_robot
Verified button codes for your specific controller.

BASE mode:  L-stick Up/Down=drive  L-stick Left/Right=rotate CCW/CW
            R-stick Up/Down=drive  R-stick Left/Right=strafe LEFT/RIGHT
            Triangle/X/Circle/Square = diagonal RF/LB/LF/RB
            R1=speed up  L1=speed down       (BASE:SPEED:0-255)
            R2=watchdog up  L2=watchdog down (BASE:WATCHDOG:ms)
            D-pad = step movement (short pulse then auto-stop)
            Options=AUTO/MANUAL  Share=emergency stop

ARM mode:   L-stick Left/Right=J0  L-stick Up/Down=J1  R-stick=J2/J3
            D-pad Left/Right=J0 (10 deg/press)  D-pad Up/Down=gripper (7 deg/press)
            R2=gripper (analog, 80-165 deg, 5 deg/step)  Triangle/X=elbow  Circle/Square=shoulder
            L1=HOME  R1=PICK sequence (ESP32 hardware auto-sequence)

R3 = toggle BASE/ARM mode
"""
import os, sys, time, logging
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi_robot.core.base_node   import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("ps4_node")

try:
    import evdev
    from evdev import InputDevice, ecodes
    EVDEV_OK = True
except ImportError:
    EVDEV_OK = False
    logger.warning("evdev not installed")

# Verified button codes for your controller
BTN_CROSS    = 304
BTN_CIRCLE   = 305
BTN_NORTH    = 307
BTN_SQUARE   = 308
BTN_L1       = 310
BTN_R1       = 311
BTN_L2_BTN   = 312
BTN_R2_BTN   = 313
BTN_SELECT   = 314   # Share
BTN_START    = 315   # Options
BTN_THUMBL   = 317   # L3
BTN_THUMBR   = 318   # R3

ABS_LX    = 0
ABS_LY    = 1
ABS_L2    = 2
ABS_RX    = 3
ABS_RY    = 4
ABS_RZ    = 5
ABS_HAT_X = 16
ABS_HAT_Y = 17

DEADZONE  = 0.15
HZ        = 20
INTERVAL  = 1.0 / HZ
STEP_DEG  = 15.0
ARM_MIN   = -90.0
ARM_MAX   =  90.0

# D-pad step sizes (per user request)
DPAD_GRIP_STEP = 7    # Up/Down  -> gripper, degrees per press
DPAD_J0_STEP   = 10.0 # Left/Right -> J0, degrees per press

# Gripper range (matches ESP32 firmware GRIP_MIN/GRIP_MAX)
GRIP_MIN = 0
GRIP_MAX = 180

# ── BASE mode: speed (R1/L1) ─────────────────────────────────────────────
SPEED_MIN  = 0
SPEED_MAX  = 255
SPEED_STEP = 25     # value step per R1/L1 press

# ── BASE mode: watchdog (R2/L2) ──────────────────────────────────────────
WATCHDOG_MIN     = 50
WATCHDOG_MAX     = 5000
WATCHDOG_STEP    = 100  # ms step per R2/L2 press
WATCHDOG_DEFAULT = 600

# ── BASE mode: D-pad step movement ───────────────────────────────────────
DPAD_STEP_MS = 200   # pulse duration before auto BASE:STOP

# ── ARM mode: R2-analog gripper (close 165->80 deg, 5 deg/step) ─────────
ARM_GRIP_CLOSED = 80
ARM_GRIP_OPEN   = 165
ARM_GRIP_STEP   = 5


def _norm(raw, lo=0, hi=255):
    mid = (hi + lo) / 2.0
    half = (hi - lo) / 2.0
    return (raw - mid) / half if half else 0.0


def _dz(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)


def _pct_esp(pct):
    return int(max(0, min(255, pct * 255 / 100)))


def _find_ds4():
    if not EVDEV_OK:
        return None
    keywords = ("wireless controller", "sony", "dualshock", "ps4")
    skip = ("touchpad", "motion", "accelero")
    try:
        devices = [InputDevice(p) for p in evdev.list_devices()]
    except Exception as e:
        logger.error(f"evdev error: {e}")
        return None
    for dev in devices:
        name = dev.name.lower()
        if any(k in name for k in keywords) and not any(s in name for s in skip):
            logger.info(f"Found DS4: {dev.name} @ {dev.path}")
            return dev
    logger.warning("No DS4 found — retry in 5s")
    return None


class PS4Node(BaseNode):

    def __init__(self, **kwargs):
        super().__init__(name="ps4_node", **kwargs)
        self._ctrl = "BASE"
        self._sys  = cfg("system", "mode", "MANUAL").upper()
        self._dev  = None
        self._lx = self._ly = self._rx = self._ry = self._l2 = self._r2 = 0.0
        self._j = [0.0, 0.0, 0.0, 0.0]
        self._g = 90
        self._last_t = 0.0
        self._base_moving = False
        self._base_spd_pct = cfg("manual", "keyboard_speed", 50)   # legacy % (used for sticks)
        self._turn_spd = cfg("manual", "keyboard_turn", 40)

        # BASE: R1/L1 stepped speed value (0-255, sent directly via BASE:SPEED)
        self._base_speed_val = _pct_esp(self._base_spd_pct)

        # BASE: R2/L2 stepped watchdog (ms)
        self._watchdog_ms = WATCHDOG_DEFAULT

        # BASE: which diagonal buttons are currently held (so we can stop on release)
        self._diag_held = set()

        # BASE: D-pad step-movement timer (auto BASE:STOP after a short pulse)
        self._dpad_stop_at = None

        # ARM: R2-analog gripper tracking (close 165->80, open 80->165, 5deg steps)
        self._arm_grip_deg = ARM_GRIP_OPEN
        self._r2_btn_prev = 0.0  # last R2 axis value sampled for step thresholding

    def setup(self):
        self._print_map()

    def run(self):
        logger.info("ps4_node running")
        while self.running:
            if self._dev is None:
                self._dev = _find_ds4()
                if self._dev is None:
                    self.sleep(5.0)
                    continue
                self._announce()
            try:
                self._loop()
            except OSError as e:
                logger.warning(f"DS4 disconnected: {e}")
                self._dev = None
                self._reset()

    def _loop(self):
        import select
        fd = self._dev.fd
        while self.running:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                try:
                    for ev in self._dev.read():
                        self._event(ev)
                except BlockingIOError:
                    pass
            now = time.time()
            if now - self._last_t >= INTERVAL:
                self._tick()
                self._last_t = now

    def _event(self, ev):
        if ev.type == ecodes.EV_ABS:
            self._axis(ev.code, ev.value)
        elif ev.type == ecodes.EV_KEY:
            if ev.value == 1:
                self._button(ev.code)
            elif ev.value == 0:
                self._button_release(ev.code)

    def _axis(self, code, raw):
        if code == ABS_LX:
            self._lx = _dz(_norm(raw))
        elif code == ABS_LY:
            self._ly = _dz(-_norm(raw))
        elif code == ABS_RX:
            self._rx = _dz(_norm(raw))
        elif code == ABS_RY:
            self._ry = _dz(-_norm(raw))
        elif code == ABS_L2:
            self._l2 = raw / 255.0
        elif code == ABS_RZ:
            self._r2 = raw / 255.0
        elif code == ABS_HAT_X:
            self._dpad_x(raw)
        elif code == ABS_HAT_Y:
            self._dpad_y(raw)

    def _dpad_x(self, val):
        if self._ctrl == "BASE":
            if val == -1:
                self._dpad_step("BASE:ROTATECCW")
            elif val == 1:
                self._dpad_step("BASE:ROTATECW")
        else:
            # ARM mode: D-pad Left/Right -> J0, 10 deg per press
            if val != 0:
                self._step(0, val * DPAD_J0_STEP)

    def _dpad_y(self, val):
        if self._ctrl == "BASE":
            if val == -1:
                self._dpad_step("BASE:FORWARD")
            elif val == 1:
                self._dpad_step("BASE:BACKWARD")
        else:
            # ARM mode: D-pad Up/Down -> gripper, 7 deg per press
            # Hat convention: -1 = Up, +1 = Down (matches BASE FORWARD/BACKWARD above)
            if val == -1:
                self._step_gripper(+DPAD_GRIP_STEP)
            elif val == 1:
                self._step_gripper(-DPAD_GRIP_STEP)

    def _dpad_step(self, cmd):
        """D-PAD (STEP MOVEMENT): send the move, then auto BASE:STOP after a short pulse."""
        self._pub(f"BASE:SPEED:{self._base_speed_val}")
        self._pub(cmd)
        self._dpad_stop_at = time.time() + (DPAD_STEP_MS / 1000.0)

    def _button(self, code):
        # Universal buttons
        if code == BTN_THUMBR:
            self._toggle_ctrl()
            return

        if code == BTN_SELECT:
            logger.warning("Share — emergency stop")
            self._pub("BASE:STOP")
            self._pub("ARM:HOME")
            self._j = [0.0, 0.0, 0.0, 0.0]
            self._g = 90
            self._diag_held.clear()
            return

        # BASE mode buttons
        if self._ctrl == "BASE":
            if code == BTN_NORTH:        # Triangle -> RF (Forward Right Diagonal)
                self._diag_press(code, "BASE:RF")
            elif code == BTN_CROSS:      # X -> LB (Backward Left Diagonal)
                self._diag_press(code, "BASE:LB")
            elif code == BTN_CIRCLE:     # Circle -> LF (Forward Left Diagonal)
                self._diag_press(code, "BASE:LF")
            elif code == BTN_SQUARE:     # Square -> RB (Backward Right Diagonal)
                self._diag_press(code, "BASE:RB")
            elif code == BTN_R1:
                self._base_speed_val = min(SPEED_MAX, self._base_speed_val + SPEED_STEP)
                self._pub(f"BASE:SPEED:{self._base_speed_val}")
                logger.info(f"Speed up → {self._base_speed_val}")
                print(f"\n[PS4] Speed → {self._base_speed_val}\n")
            elif code == BTN_L1:
                self._base_speed_val = max(SPEED_MIN, self._base_speed_val - SPEED_STEP)
                self._pub(f"BASE:SPEED:{self._base_speed_val}")
                logger.info(f"Speed down → {self._base_speed_val}")
                print(f"\n[PS4] Speed → {self._base_speed_val}\n")
            elif code == BTN_R2_BTN:
                self._watchdog_ms = min(WATCHDOG_MAX, self._watchdog_ms + WATCHDOG_STEP)
                self._pub(f"BASE:WATCHDOG:{self._watchdog_ms}")
                logger.info(f"Watchdog up → {self._watchdog_ms}ms")
                print(f"\n[PS4] Watchdog → {self._watchdog_ms}ms\n")
            elif code == BTN_L2_BTN:
                self._watchdog_ms = max(WATCHDOG_MIN, self._watchdog_ms - WATCHDOG_STEP)
                self._pub(f"BASE:WATCHDOG:{self._watchdog_ms}")
                logger.info(f"Watchdog down → {self._watchdog_ms}ms")
                print(f"\n[PS4] Watchdog → {self._watchdog_ms}ms\n")
            elif code == BTN_START:
                new = "MANUAL" if self._sys == "AUTO" else "AUTO"
                self._sys = new
                self.publish(Topics.SYSTEM_MODE, mode=new)
                print(f"\n[PS4] System → {new}\n")

        # ARM mode buttons
        else:
            if code == BTN_NORTH:
                self._step(2, +STEP_DEG)
            elif code == BTN_CROSS:
                self._step(2, -STEP_DEG)
            elif code == BTN_CIRCLE:
                self._step(1, +STEP_DEG)
            elif code == BTN_SQUARE:
                self._step(1, -STEP_DEG)
            elif code == BTN_L1:
                logger.info("L1 → HOME")
                self._j = [0.0, 0.0, 0.0, 0.0]
                self._g = 90
                self._arm_grip_deg = ARM_GRIP_OPEN
                self._pub("ARM:HOME")
            elif code == BTN_R1:
                logger.info("R1 → RUNSEQ 1 (ESP32 hardware sequence)")
                self._pub("ARM:PICK")
            elif code == BTN_THUMBL:
                logger.info("L3 → PI_SEQ (Pi-side move/delay/grip sequence)")
                self._pub("ARM:PI_SEQ")

    def _button_release(self, code):
        # BASE mode: diagonal buttons stop the base when released
        if self._ctrl == "BASE" and code in (BTN_NORTH, BTN_CROSS, BTN_CIRCLE, BTN_SQUARE):
            self._diag_held.discard(code)
            if not self._diag_held:
                self._pub("BASE:STOP")

    def _diag_press(self, code, cmd):
        self._diag_held.add(code)
        self._pub(f"BASE:SPEED:{self._base_speed_val}")
        self._pub(cmd)

    def _tick(self):
        # D-pad step-movement: auto BASE:STOP once the pulse window elapses
        if self._dpad_stop_at is not None and time.time() >= self._dpad_stop_at:
            self._pub("BASE:STOP")
            self._dpad_stop_at = None

        if self._ctrl == "BASE":
            self._tick_base()
        else:
            self._tick_arm()

    def _tick_base(self):
        # Left stick: Up/Down = drive, Left/Right = rotate CCW/CW
        # Right stick: Up/Down = drive, Left/Right = strafe LEFT/RIGHT
        # Diagonal buttons (held) take priority and are handled in _button/_button_release
        if self._diag_held:
            return

        if abs(self._ly) > 0:
            cmd = "BASE:FORWARD" if self._ly > 0 else "BASE:BACKWARD"
            self._base_cmd(cmd)
            self._base_moving = True
        elif abs(self._ry) > 0:
            cmd = "BASE:FORWARD" if self._ry > 0 else "BASE:BACKWARD"
            self._base_cmd(cmd)
            self._base_moving = True
        elif abs(self._rx) > 0:
            cmd = "BASE:RIGHT" if self._rx > 0 else "BASE:LEFT"
            self._base_cmd(cmd)
            self._base_moving = True
        elif abs(self._lx) > 0:
            cmd = "BASE:ROTATECW" if self._lx > 0 else "BASE:ROTATECCW"
            self._base_cmd(cmd)
            self._base_moving = True
        else:
            if self._base_moving:
                self._pub("BASE:STOP")
                self._base_moving = False

    def _tick_arm(self):
        rate = 90.0 * INTERVAL
        changed = False
        # Left analog Left/Right -> J0
        if abs(self._lx) > 0:
            self._j[0] = self._clamp(self._j[0] + self._lx * rate)
            changed = True
        # Left analog Up/Down -> J1
        if abs(self._ly) > 0:
            self._j[1] = self._clamp(self._j[1] + self._ly * rate)
            changed = True
        if abs(self._ry) > 0:
            self._j[2] = self._clamp(self._j[2] + self._ry * rate)
            changed = True
        if abs(self._rx) > 0:
            self._j[3] = self._clamp(self._j[3] + self._rx * rate)
            changed = True

        # R2 controls gripper: close 165->80, open 80->165, 5deg per step
        target = ARM_GRIP_CLOSED + self._r2 * (ARM_GRIP_OPEN - ARM_GRIP_CLOSED)
        steps = round((target - self._arm_grip_deg) / ARM_GRIP_STEP)
        if steps != 0:
            self._arm_grip_deg = max(ARM_GRIP_CLOSED, min(ARM_GRIP_OPEN,
                                      self._arm_grip_deg + steps * ARM_GRIP_STEP))
            self._g = self._arm_grip_deg
            changed = True

        if changed:
            frame = f"ARM:COMPACT:${self._j[0]:.1f},{self._j[1]:.1f},{self._j[2]:.1f},{self._j[3]:.1f},{self._g}"
            self._pub(frame)

    def _step(self, j, delta):
        self._j[j] = self._clamp(self._j[j] + delta)
        frame = f"ARM:COMPACT:${self._j[0]:.1f},{self._j[1]:.1f},{self._j[2]:.1f},{self._j[3]:.1f},{self._g}"
        self._pub(frame)

    def _step_gripper(self, delta):
        self._g = max(GRIP_MIN, min(GRIP_MAX, self._g + delta))
        frame = f"ARM:COMPACT:${self._j[0]:.1f},{self._j[1]:.1f},{self._j[2]:.1f},{self._j[3]:.1f},{self._g}"
        self._pub(frame)

    def _clamp(self, d):
        return max(ARM_MIN, min(ARM_MAX, d))

    def _base_cmd(self, cmd):
        self._pub(f"BASE:SPEED:{self._base_speed_val}")
        self._pub(cmd)

    def _toggle_ctrl(self):
        self._ctrl = "ARM" if self._ctrl == "BASE" else "BASE"
        if self._ctrl == "ARM":
            self._pub("BASE:STOP")
            self._diag_held.clear()
            self._dpad_stop_at = None
        self._announce()

    def _announce(self):
        print(f"\n[PS4] -- {self._ctrl} MODE -- (R3 to switch)\n")
        logger.info(f"Controller → {self._ctrl}")

    def _pub(self, cmd):
        logger.debug(f"PS4 → {cmd}")
        self.publish(Topics.MANUAL_COMMAND, command=cmd)

    def _reset(self):
        self._lx = self._ly = self._rx = self._ry = self._l2 = self._r2 = 0.0
        self._base_moving = False
        self._diag_held.clear()
        self._dpad_stop_at = None

    def _print_map(self):
        print("\n" + "=" * 52)
        print("  PS4 Controller -- pi_robot")
        print("=" * 52)
        print("  R3       -> toggle BASE / ARM mode")
        print("  BASE:  L-stick U/D=drive  L-stick L/R=rotate")
        print("         R-stick U/D=drive  R-stick L/R=strafe")
        print("         Triangle/X/Circle/Square = RF/LB/LF/RB diagonal")
        print("         R1=speed+  L1=speed-   (BASE:SPEED:0-255)")
        print("         R2=watchdog+  L2=watchdog-   (BASE:WATCHDOG:ms)")
        print("         D-pad=step movement   Options=AUTO/MANUAL  Share=stop")
        print("  ARM:   L-stick L/R=J0  L-stick U/D=J1  R-stick=J2/J3")
        print("         D-pad L/R=J0 (10deg)  D-pad U/D=gripper (7deg)")
        print("         R2=gripper(analog, 80-165deg)  L1=HOME  R1=PICK(ESP32)")
        print("         L3=PI_SEQ (Pi-side move/delay/grip sequence)")
        print("=" * 52 + "\n")
