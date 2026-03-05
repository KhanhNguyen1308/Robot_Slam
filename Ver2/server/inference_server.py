#!/usr/bin/env python3
"""
x99 Inference Server — YOLOv11 Object Detection
================================================
Runs on the x99 server (192.168.2.10).  Receives JPEG frames from the robot
over LAN and returns JSON detection results.

Hardware
--------
  CPU  : Dual Xeon E5-2680v4 (28 cores / 56 threads)
  GPU  : AMD Radeon MI50 16 GB (ROCm, gfx906)
  RAM  : 32 GB

Installation
------------
  # ROCm 5.6 PyTorch (MI50 = gfx906)
  pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.6
  pip install ultralytics fastapi uvicorn opencv-python-headless numpy

  # For MI50 you may need to override the GFX version:
  export HSA_OVERRIDE_GFX_VERSION=9.0.6

Usage
-----
  # With ROCm GPU (recommended):
  HSA_OVERRIDE_GFX_VERSION=9.0.6 python3 server/inference_server.py

  # CPU-only fallback:
  python3 server/inference_server.py --device cpu

  # Custom model / port:
  python3 server/inference_server.py --model yolo11x.pt --port 8090

Available YOLOv11 models (tradeoff: speed vs accuracy on MI50):
  yolo11n.pt  ~  2 ms/frame   (nano   — fastest)
  yolo11s.pt  ~  4 ms/frame   (small)
  yolo11m.pt  ~  8 ms/frame   (medium — recommended default)
  yolo11l.pt  ~ 14 ms/frame   (large)
  yolo11x.pt  ~ 22 ms/frame   (extra-large — most accurate)

API
---
  GET  /health        — liveness check + stats
  POST /detect        — body: raw JPEG bytes
                        response: {"detections": [...], "inference_ms": float}
"""

import argparse
import io
import logging
import time
from typing import List

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s: %(message)s",
)
logger = logging.getLogger("inference_server")


# ---------------------------------------------------------------------------
# CLI arguments
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="YOLOv11 inference server for robot")
parser.add_argument("--model",  default="yolo11m.pt",
                    help="YOLO model weights file (downloaded automatically on first run)")
parser.add_argument("--device", default="auto",
                    help="Inference device: 'auto', 'cpu', or '0' (first GPU)")
parser.add_argument("--host",   default="0.0.0.0")
parser.add_argument("--port",   type=int,   default=8090)
parser.add_argument("--conf",   type=float, default=0.35,
                    help="Minimum confidence threshold")
parser.add_argument("--iou",    type=float, default=0.45,
                    help="NMS IoU threshold")
parser.add_argument("--imgsz",  type=int,   default=640,
                    help="Inference input size (pixels, square)")
args, _ = parser.parse_known_args()


# ---------------------------------------------------------------------------
# Device selection
# ---------------------------------------------------------------------------
def _select_device() -> str:
    if args.device != "auto":
        return args.device
    try:
        import torch
        if torch.cuda.is_available():
            name = torch.cuda.get_device_name(0)
            vram = torch.cuda.get_device_properties(0).total_memory / 1024**3
            logger.info(f"GPU detected: {name}  ({vram:.1f} GB VRAM)")
            return "0"
    except ImportError:
        pass
    logger.warning("No CUDA/ROCm GPU detected — falling back to CPU inference")
    return "cpu"

DEVICE = _select_device()


# ---------------------------------------------------------------------------
# Load model
# ---------------------------------------------------------------------------
logger.info(f"Loading {args.model} on device={DEVICE!r} …")
model = YOLO(args.model)
model.to(DEVICE)

# Warm-up: run a dummy inference so the first real request is not slow
_dummy = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)
model(_dummy, imgsz=args.imgsz, conf=args.conf, verbose=False)
logger.info("Model warm-up complete — server ready")


# ---------------------------------------------------------------------------
# FastAPI application
# ---------------------------------------------------------------------------
app = FastAPI(
    title="Robot Vision Server",
    description="YOLOv11 object detection for autonomous robot",
    version="1.0.0",
)

_stats = {
    "requests": 0,
    "total_inference_ms": 0.0,
    "errors": 0,
    "started_at": time.time(),
}


@app.get("/health")
def health():
    """Liveness + performance statistics endpoint."""
    n = max(_stats["requests"], 1)
    return {
        "status": "ok",
        "model": args.model,
        "device": DEVICE,
        "imgsz": args.imgsz,
        "conf_threshold": args.conf,
        "requests": _stats["requests"],
        "errors": _stats["errors"],
        "avg_inference_ms": round(_stats["total_inference_ms"] / n, 1),
        "uptime_s": round(time.time() - _stats["started_at"], 1),
    }


@app.post("/detect")
async def detect(request: Request):
    """
    Receive a JPEG image and return YOLO detection results.

    Request body : raw JPEG bytes  (Content-Type: image/jpeg)
    Response     : JSON
        {
          "detections": [
            {
              "class_id":   int,
              "label":      str,
              "confidence": float,   // 0–1
              "bbox":       [x1, y1, x2, y2]  // pixel coords in received image
            },
            ...
          ],
          "count":        int,
          "inference_ms": float,
          "frame_size":   [width, height]
        }
    """
    body = await request.body()
    if not body:
        _stats["errors"] += 1
        raise HTTPException(status_code=400, detail="Empty request body")

    # ---- Decode JPEG --------------------------------------------------------
    try:
        buf   = np.frombuffer(body, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("cv2.imdecode returned None")
    except Exception as exc:
        _stats["errors"] += 1
        raise HTTPException(status_code=400, detail=f"Could not decode image: {exc}")

    h, w = frame.shape[:2]

    # ---- Inference ----------------------------------------------------------
    t0 = time.perf_counter()
    results = model(
        frame,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        verbose=False,
    )
    inference_ms = (time.perf_counter() - t0) * 1000.0

    _stats["requests"] += 1
    _stats["total_inference_ms"] += inference_ms

    # ---- Parse results ------------------------------------------------------
    detections: List[dict] = []
    for r in results:
        boxes = r.boxes
        if boxes is None:
            continue
        for box in boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = model.names.get(cls, str(cls))
            detections.append({
                "class_id":   cls,
                "label":      label,
                "confidence": round(conf, 4),
                "bbox":       [x1, y1, x2, y2],
            })

    logger.debug(
        f"detect: {len(detections)} objects in {inference_ms:.1f} ms "
        f"(frame {w}x{h})"
    )

    return JSONResponse({
        "detections":   detections,
        "count":        len(detections),
        "inference_ms": round(inference_ms, 1),
        "frame_size":   [w, h],
    })


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    logger.info(
        f"Starting inference server on {args.host}:{args.port}  "
        f"model={args.model}  device={DEVICE}  imgsz={args.imgsz}"
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
