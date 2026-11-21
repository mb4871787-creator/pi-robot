"""
camera_vision_node.py
---------------------
Replaces camera_node.py + vision_node.py in one node.

Captures Pi camera frames, detects QR codes, detects cube
color (RED/GREEN/BLUE) around the QR bounding box.

Publishes:
    Topics.CAMERA_FRAME       -> { "frame": np.ndarray }
    Topics.VISION_QR_DETECTED -> { qr_data, bbox, center_x, center_y,
                                    area, error_x, error_y, frame_w, frame_h,
                                    color, color_rgb, confidence }
    Topics.VISION_QR_LOST     -> {}
"""
import cv2, time, numpy as np, logging, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pi_robot.core.base_node import BaseNode
from pi_robot.core.message_bus import Topics
from config.config_loader import get as cfg

logger = logging.getLogger("camera_vision_node")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

COLOR_RANGES = {
    "RED":   [(np.array([0,  100,80]), np.array([10, 255,255])),
              (np.array([165,100,80]), np.array([180,255,255]))],
    "GREEN": [(np.array([40, 80, 60]), np.array([90, 255,255]))],
    "BLUE":  [(np.array([95, 80, 60]), np.array([135,255,255]))],
}
COLOR_RGB = {
    "RED":     [220, 60,  60],
    "GREEN":   [60,  200, 80],
    "BLUE":    [60,  120, 220],
    "UNKNOWN": [120, 120, 120],
}
COLOR_MIN_RATIO = 0.15
RETRY_INTERVAL  = 5.0


class CameraVisionNode(BaseNode):

    def __init__(self, **kwargs):
        super().__init__(name="camera_vision_node", **kwargs)
        self._width      = cfg("camera", "frame_width",  640)
        self._height     = cfg("camera", "frame_height", 480)
        self._fps        = cfg("camera", "fps",           30)
        self._min_area   = cfg("vision", "min_qr_area",  500)
        self._picam      = None
        self._detector   = None
        self._was_detected = False
        self._period     = 1.0 / self._fps

    def setup(self):
        self._detector = cv2.QRCodeDetector()
        logger.info(f"camera_vision_node ready — "
                    f"{self._width}x{self._height} @ {self._fps}fps")

    def run(self):
        logger.info("camera_vision_node running")
        while self.running:
            if not PICAMERA2_AVAILABLE:
                logger.error("picamera2 not available — install it first")
                self.sleep(10.0)
                continue
            self._run_camera()
            if self.running:
                logger.warning(f"Camera stopped — retrying in {RETRY_INTERVAL}s")
                self.sleep(RETRY_INTERVAL)

    def cleanup(self):
        self._release()

    def _run_camera(self):
        try:
            self._picam = Picamera2()
            config = self._picam.create_preview_configuration(
                main={"size": (self._width, self._height), "format": "RGB888"})
            self._picam.configure(config)
            self._picam.set_controls({"FrameRate": float(self._fps)})
            self._picam.start()
            logger.info(f"Pi camera started — "
                        f"{self._width}x{self._height} @ {self._fps}fps")
        except Exception as e:
            logger.error(f"Camera open failed: {e}")
            self._release()
            return

        last = time.time()
        while self.running:
            try:
                rgb   = self._picam.capture_array()
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

                # Publish raw frame
                self.publish(Topics.CAMERA_FRAME, frame=frame)

                # Detect QR + color
                self._process(frame)

                # Throttle to FPS
                elapsed = time.time() - last
                sleep_t = self._period - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                last = time.time()

            except Exception as e:
                logger.warning(f"Frame error: {e}")
                self._release()
                return

    def _process(self, frame):
        fh, fw = frame.shape[:2]
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        data, bbox, _ = self._detector.detectAndDecode(gray)

        if data and bbox is not None:
            pts  = bbox[0].astype(int)
            cx   = float(np.mean(pts[:, 0]))
            cy   = float(np.mean(pts[:, 1]))
            area = float(cv2.contourArea(pts))

            if area < self._min_area:
                logger.debug(f"QR too small area={area:.0f} — ignored")
                self._publish_lost()
                return

            color, conf, rgb = self._detect_color(frame, pts)

            logger.debug(f"QR='{data}' color={color}({conf:.2f}) "
                         f"cx={cx:.0f} cy={cy:.0f} area={area:.0f}")

            self.publish(
                Topics.VISION_QR_DETECTED,
                qr_data    = data,
                bbox       = pts.tolist(),
                center_x   = cx,
                center_y   = cy,
                area       = area,
                error_x    = cx - fw / 2.0,
                error_y    = cy - fh / 2.0,
                frame_w    = fw,
                frame_h    = fh,
                color      = color,
                color_rgb  = rgb,
                confidence = conf,
            )
            self._was_detected = True

        else:
            if self._was_detected:
                self._publish_lost()
            self._was_detected = False

    def _detect_color(self, frame, pts):
        try:
            xmin, xmax = int(np.min(pts[:,0])), int(np.max(pts[:,0]))
            ymin, ymax = int(np.min(pts[:,1])), int(np.max(pts[:,1]))
            pw = int((xmax - xmin) * 0.4)
            ph = int((ymax - ymin) * 0.4)
            fh, fw = frame.shape[:2]
            rx1, ry1 = max(0, xmin-pw), max(0, ymin-ph)
            rx2, ry2 = min(fw, xmax+pw), min(fh, ymax+ph)
            region   = frame[ry1:ry2, rx1:rx2]
            if region.size == 0:
                return "UNKNOWN", 0.0, COLOR_RGB["UNKNOWN"]

            # Mask out the QR code itself
            mask = np.ones(region.shape[:2], dtype=np.uint8) * 255
            iy1 = max(0, ymin-ry1); iy2 = max(0, ymax-ry1)
            ix1 = max(0, xmin-rx1); ix2 = max(0, xmax-rx1)
            mask[iy1:iy2, ix1:ix2] = 0

            hsv   = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
            total = max(1, int(np.count_nonzero(mask)))

            best, best_r = "UNKNOWN", 0.0
            for name, ranges in COLOR_RANGES.items():
                m = np.zeros(region.shape[:2], dtype=np.uint8)
                for lo, hi in ranges:
                    m |= cv2.inRange(hsv, lo, hi)
                m &= mask
                r = int(np.count_nonzero(m)) / total
                if r > best_r:
                    best_r, best = r, name

            if best_r < COLOR_MIN_RATIO:
                best = "UNKNOWN"
            return best, round(best_r, 2), COLOR_RGB[best]

        except Exception as e:
            logger.debug(f"Color error: {e}")
            return "UNKNOWN", 0.0, COLOR_RGB["UNKNOWN"]

    def _publish_lost(self):
        logger.info("QR lost")
        self.publish(Topics.VISION_QR_LOST)
        self._was_detected = False

    def _release(self):
        if self._picam:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
            self._picam = None
            logger.info("Pi camera released")
