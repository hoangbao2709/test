# -*- coding: utf-8 -*-
import time
import numpy as np
from flask import Response, stream_with_context
from . import config

cv2 = None
_cap = None


def _open_camera():
    """M? camera; th? CAP_V4L2 tru?c r?i fallback CAP_ANY.
    Kh�ng �p FOURCC n?u thi?t b? kh�ng h? tr?."""
    global cv2, _cap
    try:
        import cv2 as _cv2
        cv2 = _cv2

        cam = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
        if not cam.isOpened():
            cam = cv2.VideoCapture(config.CAMERA_INDEX)  # CAP_ANY

        if not cam.isOpened():
            print(f"[Camera] Cannot open index {config.CAMERA_INDEX}")
            _cap = None
            return

        cam.set(cv2.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
        cam.set(cv2.CAP_PROP_FPS,          config.FRAME_FPS)
        try:
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass

        _cap = cam
        print(f"[Camera] Opened index {config.CAMERA_INDEX} ({config.FRAME_W}x{config.FRAME_H} @ {config.FRAME_FPS}fps)")
    except Exception as e:
        print("[Camera] Init error:", e)
        _cap = None


def init_camera():
    global _cap
    if _cap is None:
        _open_camera()


def _blank_jpeg():
    """T?o 1 khung den d? stream kh�ng b? treo khi thi?u frame."""
    if cv2 is None:
        return b""
    img = np.zeros((config.FRAME_H, config.FRAME_W, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else b""


def mjpeg_generator():
    """
    Multipart MJPEG chu?n:
    --frame\r\n
    Content-Type: image/jpeg\r\n
    Content-Length: N\r\n
    \r\n
    <JPEG bytes>\r\n
    """
    boundary = b"--frame"
    if _cap is None or cv2 is None:
        jpg = _blank_jpeg()
        while True:
            header = boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: " \
                     + str(len(jpg)).encode() + b"\r\n\r\n"
            yield header + jpg + b"\r\n"
            time.sleep(0.5)
    else:
        while True:
            ok, frame = _cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            jpg = buf.tobytes()
            header = boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: " \
                     + str(len(jpg)).encode() + b"\r\n\r\n"
            yield header + jpg + b"\r\n"


def camera_response():
    """Tr? MJPEG stream v?i header no-cache/keep-alive + t?t buffer proxy."""
    headers = {
        "Cache-Control": "no-cache, no-store, must-revalidate",
        "Pragma": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",  # Nginx proxy: t?t buffering
    }
    return Response(
        stream_with_context(mjpeg_generator()),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers=headers,
        direct_passthrough=True,
    )


def single_frame_response():
    """Tr? 1 ?nh JPEG (fallback / test)."""
    if _cap is None or cv2 is None:
        jpg = _blank_jpeg()
    else:
        ok, frame = _cap.read()
        if not ok or frame is None:
            jpg = _blank_jpeg()
        else:
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            jpg = buf.tobytes() if ok else _blank_jpeg()
    return Response(jpg, mimetype="image/jpeg")


def cleanup_camera():
    try:
        if _cap is not None:
            _cap.release()
    except Exception:
        pass
