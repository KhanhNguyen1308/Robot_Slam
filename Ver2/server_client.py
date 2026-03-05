#!/usr/bin/env python3
"""
Vision Server Client
Streams frames from the 4K UGREEN webcam (device 2) to the x99 inference server
and receives YOLOv11 object detections back.

Runs entirely in a background thread.  All public methods return immediately —
the robot safety layer never stalls waiting for a network response.

Usage (in main.py):
    from server_client import VisionServerClient
    client = VisionServerClient(device_id=2, server_host='192.168.2.10')
    client.start()
    detections = client.get_latest_detections()   # non-blocking
    client.stop()
"""

import cv2
import json
import logging
import math
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    """A single YOLO object detection result."""
    class_id: int
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]       # x1, y1, x2, y2 in stream-resolution pixels
    center_x: float                        # normalised [0, 1] across stream width
    center_y: float                        # normalised [0, 1] across stream height


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class VisionServerClient:
    """
    Captures frames from the 4K UGREEN webcam, downscales for the network,
    POSTs them to the x99 inference server, and stores the latest YOLO
    detection results for the rest of the robot stack to consume.

    Safety contract
    ---------------
    * ``get_latest_detections()`` and ``get_latest_frame()`` are non-blocking.
    * If the server is unreachable or times out the last known detections are
      kept until a successful response arrives; ``is_server_available`` becomes
      False so callers can react gracefully.
    * Camera capture errors do not raise — they are logged and retried.
    """

    def __init__(self,
                 device_id: int = 2,
                 capture_width: int = 3840,
                 capture_height: int = 2160,
                 capture_fps: int = 15,
                 stream_width: int = 1280,
                 stream_height: int = 720,
                 jpeg_quality: int = 85,
                 server_host: str = "192.168.2.10",
                 server_port: int = 8090,
                 timeout_s: float = 0.15,
                 min_confidence: float = 0.40):

        self.device_id = device_id
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.capture_fps = capture_fps
        self.stream_width = stream_width
        self.stream_height = stream_height
        self.jpeg_quality = jpeg_quality
        self.server_url = f"http://{server_host}:{server_port}/detect"
        self.timeout_s = timeout_s
        self.min_confidence = min_confidence

        # Camera handle
        self._cap: Optional[cv2.VideoCapture] = None

        # Shared state (protected by _lock)
        self._lock = threading.Lock()
        self._latest_frame: Optional[object] = None    # stream-sized BGR ndarray
        self._latest_detections: List[Detection] = []
        self._server_available: bool = False
        self._frames_sent: int = 0
        self._detections_received: int = 0
        self._last_server_error: str = ""

        # Thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Open the camera and start the background capture+inference thread."""
        if self._running:
            return True
        if not self._open_camera():
            return False
        self._running = True
        self._thread = threading.Thread(
            target=self._inference_loop, daemon=True, name="vision_client"
        )
        self._thread.start()
        logger.info(
            f"VisionServerClient started — camera /dev/video{self.device_id}, "
            f"server {self.server_url}, "
            f"stream {self.stream_width}x{self.stream_height}"
        )
        return True

    def stop(self):
        """Stop the background thread and release the camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._cap:
            self._cap.release()
            self._cap = None
        logger.info("VisionServerClient stopped")

    def get_latest_detections(self) -> List[Detection]:
        """Return the most recent detection list (thread-safe, non-blocking)."""
        with self._lock:
            return list(self._latest_detections)

    def get_latest_frame(self):
        """Return the latest stream-resolution BGR frame (thread-safe, non-blocking).

        Returns None if no frame has been captured yet.
        """
        with self._lock:
            if self._latest_frame is None:
                return None
            import numpy as np
            return self._latest_frame.copy()

    @property
    def is_server_available(self) -> bool:
        with self._lock:
            return self._server_available

    def get_stats(self) -> Dict:
        """Return diagnostics dict (non-blocking)."""
        with self._lock:
            return {
                "camera_device": self.device_id,
                "server_url": self.server_url,
                "server_available": self._server_available,
                "frames_sent": self._frames_sent,
                "detections_received": self._detections_received,
                "detections_count": len(self._latest_detections),
                "last_error": self._last_server_error,
            }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _open_camera(self) -> bool:
        """Open the UGREEN 4K USB webcam."""
        cap = cv2.VideoCapture(self.device_id)
        # Request MJPEG from the camera itself to reduce USB 3.0 bandwidth
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        cap.set(cv2.CAP_PROP_FPS,          self.capture_fps)

        if not cap.isOpened():
            logger.error(
                f"VisionServerClient: failed to open camera {self.device_id}. "
                f"Check that /dev/video{self.device_id} exists and is not in use."
            )
            return False

        actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        logger.info(
            f"Object camera opened: {actual_w}x{actual_h} @ {actual_fps:.0f} fps "
            f"(device {self.device_id})"
        )
        self._cap = cap
        return True

    def _inference_loop(self):
        """Capture → downscale → JPEG encode → POST → parse → store."""
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        while self._running:
            # ---- capture ------------------------------------------------
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.5)
                continue

            ret, frame = self._cap.read()
            if not ret:
                logger.warning("Object camera: failed to read frame, retrying…")
                time.sleep(0.1)
                continue

            # ---- downscale for network ----------------------------------
            stream_frame = cv2.resize(
                frame,
                (self.stream_width, self.stream_height),
                interpolation=cv2.INTER_AREA,
            )

            # ---- JPEG encode --------------------------------------------
            ok, buf = cv2.imencode('.jpg', stream_frame, encode_params)
            if not ok:
                continue
            jpeg_bytes = buf.tobytes()

            # ---- POST to server (tight timeout) -------------------------
            detections = self._post_frame(jpeg_bytes)

            # ---- store results atomically --------------------------------
            with self._lock:
                self._latest_frame = stream_frame
                self._frames_sent += 1
                if detections is not None:
                    self._latest_detections = detections
                    self._server_available = True
                    self._detections_received += 1
                else:
                    # Keep last detections stale; caller reads is_server_available
                    self._server_available = False

    def _post_frame(self, jpeg_bytes: bytes) -> Optional[List[Detection]]:
        """POST JPEG bytes → return parsed Detection list, or None on error."""
        try:
            req = urllib.request.Request(
                self.server_url,
                data=jpeg_bytes,
                headers={"Content-Type": "image/jpeg"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read()

            data = json.loads(body)
            detections: List[Detection] = []
            for d in data.get("detections", []):
                if d["confidence"] < self.min_confidence:
                    continue
                x1, y1, x2, y2 = d["bbox"]
                detections.append(
                    Detection(
                        class_id=d["class_id"],
                        label=d["label"],
                        confidence=d["confidence"],
                        bbox=(x1, y1, x2, y2),
                        center_x=(x1 + x2) / 2.0 / self.stream_width,
                        center_y=(y1 + y2) / 2.0 / self.stream_height,
                    )
                )
            return detections

        except urllib.error.URLError as exc:
            with self._lock:
                self._last_server_error = str(exc.reason)
            return None
        except Exception as exc:
            with self._lock:
                self._last_server_error = str(exc)
            return None
