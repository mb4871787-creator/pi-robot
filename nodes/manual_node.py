"""
manual_node.py — Keyboard control node
"""
import sys, os, time, logging
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi_robot.core.base_node   import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("manual_node")

try:
    import termios, tty
    UNIX_TERMINAL = True
except ImportError:
    UNIX_TERMINAL = False


class ManualNode(BaseNode):

    KEY_MAP = {
        "w": "BASE:FORWARD",
        "s": "BASE:BACKWARD",
        "a": "BASE:ROTATECCW",
        "d": "BASE:ROTATECW",
        " ": "BASE:STOP",
        "q": "ARM:HOME",
        "e": "ARM:PICK",
        "z": "ARM:LOWER",
        "x": "ARM:RAISE",
        "c": "ARM:GRIP",
        "v": "ARM:PLACE",
    }

    def __init__(self, **kwargs):
        super().__init__(name="manual_node", **kwargs)
        self._mode = cfg("system", "mode", "MANUAL")

    def setup(self):
        logger.info("manual_node ready")
        print("\n" + "="*50)
        print("  MANUAL CONTROL — Key Bindings")
        print("="*50)
        print("  W / S  — Forward / Backward")
        print("  A / D  — Rotate Left / Right")
        print("  SPACE  — Stop")
        print("  Q      — Arm Home")
        print("  E      — Arm Pick")
        print("  Z / X  — Arm Lower / Raise")
        print("  C / V  — Grip / Place")
        print("  M      — Toggle AUTO / MANUAL")
        print("  ESC    — Emergency Stop + Quit")
        print("="*50 + "\n")

    def run(self):
        logger.info("manual_node running")
        if UNIX_TERMINAL:
            self._run_unix()
        else:
            self._run_fallback()

    def _run_unix(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = self._read_key(fd)
                if key:
                    self._handle_key(key)
                time.sleep(0.02)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print()

    def _read_key(self, fd):
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            return os.read(fd, 3).decode("utf-8", errors="ignore")
        return ""

    def _run_fallback(self):
        while self.running:
            try:
                key = input("Command> ").strip().lower()
                if key:
                    self._handle_key(key[0])
            except (EOFError, KeyboardInterrupt):
                self._handle_key("\x1b")

    def _handle_key(self, key):
        key_lower = key.lower()

        if "\x1b" in key or key == "\x03":
            logger.warning("ESC — emergency stop")
            self._send("BASE:STOP")
            self._send("ARM:HOME")
            self.running = False
            self.publish(Topics.SYSTEM_SHUTDOWN, reason="user_esc")
            return

        if "m" in key_lower:
            new = "MANUAL" if self._mode.upper() == "AUTO" else "AUTO"
            self._mode = new
            logger.info(f"Mode → {new}")
            self.publish(Topics.SYSTEM_MODE, mode=new)
            print(f"\n[MODE] → {new}\n")
            return

        for char in key_lower:
            if char in self.KEY_MAP:
                self._send(self.KEY_MAP[char])
                return

    def _send(self, cmd):
        logger.debug(f"Manual: {cmd}")
        self.publish(Topics.MANUAL_COMMAND, command=cmd)
