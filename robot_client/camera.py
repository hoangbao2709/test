# robot_client/camera.py
# -*- coding: utf-8 -*-
import threading
import time
import requests
import cv2
import numpy as np


class _OpenCVCapture:
    """
    Backend dùng OpenCV VideoCapture. Lưu ý: không truyền được headers.
    Nếu server yêu cầu API key header thì backend này có thể không dùng được,
    khi đó sẽ fallback sang HTTP MJPEG.
    """
    def __init__(self, url: str):
        self.url = url
        self.cap = None

    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.url)
        if not self.cap.isOpened():
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = cv2.VideoCapture(self.url)
        return self.cap.isOpened()

    def read(self):
        if self.cap is None:
            return False, None
        return self.cap.read()

    def release(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None


class _HTTPMjpegCapture:
    """
    Đọc MJPEG multipart/x-mixed-replace qua HTTP với headers/verify.
    Server dùng boundary=frame và Content-Type: image/jpeg.
    """
    def __init__(
        self,
        url: str,
        session: requests.Session | None = None,
        headers: dict | None = None,
        timeout: float = 5.0,
        verify_ssl: bool = True,
        chunk_size: int = 4096,
        boundary: bytes = b"--frame",
    ):
        self.url = url
        self.s = session or requests.Session()
        self.headers = headers or {}
        self.timeout = timeout
        self.verify_ssl = verify_ssl
        self.chunk_size = chunk_size
        self.boundary = boundary
        self.ct_jpeg = b"Content-Type: image/jpeg"
        self._resp = None
        self._iter = None
        self._buf = bytearray()

    def open(self) -> bool:
        try:
            self._resp = self.s.get(
                self.url,
                stream=True,
                headers=self.headers,
                timeout=self.timeout,
                verify=self.verify_ssl,
            )
            self._resp.raise_for_status()
            self._iter = self._resp.iter_content(chunk_size=self.chunk_size)
            return True
        except Exception as e:
            print("[HTTPMjpeg] open error:", e)
            self._resp = None
            self._iter = None
            return False

    def _next_jpeg_bytes(self):
        if self._iter is None:
            return None
        for chunk in self._iter:
            if not chunk:
                continue
            self._buf.extend(chunk)

            while True:
                # tìm boundary bắt đầu
                i0 = self._buf.find(self.boundary)
                if i0 < 0:
                    break
                if i0 > 0:
                    del self._buf[:i0]

                # tìm header content-type
                ih = self._buf.find(self.ct_jpeg)
                if ih < 0:
                    break

                # kết thúc header
                he = self._buf.find(b"\r\n\r\n", ih)
                if he < 0:
                    break
                data_start = he + 4

                # tìm boundary kế tiếp (kết ảnh)
                nb = self._buf.find(self.boundary, data_start)
                if nb < 0:
                    break

                # cắt ra nội dung JPEG (bỏ \r\n trước boundary)
                jpg = bytes(self._buf[data_start:nb-2])
                del self._buf[:nb]
                return jpg
        return None

    def read(self):
        jpg = self._next_jpeg_bytes()
        if jpg is None:
            return False, None
        arr = np.frombuffer(jpg, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return False, None
        return True, frame

    def release(self):
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass
        self._resp = None
        self._iter = None
        self._buf.clear()


class Camera:
    """
    Camera reader:
      - Thử OpenCV VideoCapture (không header).
      - Nếu fail (hoặc cần header/API key), fallback HTTP MJPEG (requests, có headers/verify).
    Dùng:
        cam = Camera("https://<domain>/camera", headers={"X-API-Key":"..."},
                     timeout=5, verify_ssl=True)
        cam.start()
        frame = cam.get_latest()
        cam.stop()
    """
    def __init__(
        self,
        url: str,
        headers: dict | None = None,
        timeout: float = 5.0,
        verify_ssl: bool = True,
        session: requests.Session | None = None,
        retry_sec: float = 2.0,
        prefer_opencv: bool = True,
    ):
        self.url = url
        self.headers = headers or {}
        self.timeout = float(timeout)
        self.verify_ssl = bool(verify_ssl)
        self.s = session or requests.Session()
        self.retry_sec = float(retry_sec)
        self.prefer_opencv = bool(prefer_opencv)

        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._t: threading.Thread | None = None
        self.backend: str | None = None
        self._backend_obj = None  # instance of backend to release on stop

    def start(self):
        if self._t and self._t.is_alive():
            return
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def stop(self):
        self._running = False
        # chờ thread kết thúc
        if self._t and self._t.is_alive():
            self._t.join(timeout=2.0)
        # release backend còn giữ
        try:
            if self._backend_obj:
                self._backend_obj.release()
        except Exception:
            pass
        self._backend_obj = None
        self.backend = None

    def get_latest(self):
        with self._lock:
            return self._frame

    # ---------- luồng ghi hình ----------
    def _loop(self):
        while self._running:
            # quyết định thứ tự backend
            backends = []
            if self.prefer_opencv:
                backends = ["cv", "http"]
            else:
                backends = ["http", "cv"]

            opened = False
            for b in backends:
                if not self._running:
                    break
                if b == "cv":
                    cap = _OpenCVCapture(self.url)
                    if cap.open():
                        print("[CameraClient] Using OpenCV VideoCapture")
                        self.backend = "cv"
                        self._backend_obj = cap
                        opened = True
                        while self._running:
                            ok, frame = cap.read()
                            if not ok or frame is None:
                                break
                            with self._lock:
                                self._frame = frame
                        cap.release()
                        self._backend_obj = None
                        print(f"[CameraClient] OpenCV stream ended, retrying in {self.retry_sec:.1f}s...")
                        break  # thử lại từ đầu vòng while _running

                if b == "http":
                    httpcap = _HTTPMjpegCapture(
                        url=self.url,
                        session=self.s,
                        headers=self.headers,
                        timeout=self.timeout,
                        verify_ssl=self.verify_ssl,
                    )
                    if httpcap.open():
                        print("[CameraClient] Using HTTP MJPEG fallback (requests)")
                        self.backend = "http"
                        self._backend_obj = httpcap
                        opened = True
                        while self._running:
                            ok, frame = httpcap.read()
                            if not ok or frame is None:
                                break
                            with self._lock:
                                self._frame = frame
                        httpcap.release()
                        self._backend_obj = None
                        print(f"[CameraClient] HTTP MJPEG ended, retrying in {self.retry_sec:.1f}s...")
                        break  # thử lại từ đầu vòng while _running

            if not opened:
                print(f"[CameraClient] Cannot open stream, retry in {self.retry_sec:.1f}s...")
            time.sleep(self.retry_sec)


class CameraReader(Camera):
    def __init__(self, url: str | None = None, **kwargs):
        if url is None:
            try:
                from . import config
                url = config.CAMERA_URL
                # tự động lấy headers/timeout/verify nếu có trong config
                kwargs.setdefault("headers", getattr(config, "REQUEST_HEADERS", None))
                kwargs.setdefault("timeout", getattr(config, "REQUEST_TIMEOUT", 5))
                kwargs.setdefault("verify_ssl", getattr(config, "VERIFY_SSL", True))
            except Exception:
                raise RuntimeError("CameraReader needs URL or config.CAMERA_URL")
        super().__init__(url, **kwargs)
