# -*- coding: utf-8 -*-
import threading
import time
import requests
from . import config


class CommandRepeater:
    """
    Gửi lặp lại lệnh hiện tại theo tần số REPEATER_HZ.
    Sử dụng callback _post để gửi HTTP (Session tái sử dụng).
    """
    def __init__(self, hz=config.REPEATER_HZ, post_func=None):
        self.hz = float(hz)
        self._target = None              
        self._lock = threading.Lock()
        self._running = True
        self._post = post_func or (lambda payload: (None, "no_post_func"))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set(self, command: str, **kwargs):
        """Đặt lệnh cần lặp (ví dụ: 'move', params...)."""
        with self._lock:
            self._target = (command, kwargs)

    def clear(self):
        """Xoá lệnh cần lặp (ngừng gửi lặp)."""
        with self._lock:
            self._target = None

    def stop(self):
        """Dừng thread repeater."""
        self._running = False
        self.clear()

    def _loop(self):
        period = 1.0 / max(self.hz, 0.1)
        while self._running:
            target = None
            with self._lock:
                target = self._target
            if target is not None:
                cmd, kw = target
                payload = {"command": cmd}
                payload.update(kw or {})
                self._post(payload)
            time.sleep(period)


class Control:
    """
    Lớp điều khiển gửi HTTP tới server:
      - Payload vẫn là {"command": "..."} để tương thích server hiện tại của bạn.
      - Có thể cấu hình URL/headers/timeout/verify_ssl.
    """

    def __init__(
        self,
        url: str | None = None,
        headers: dict | None = None,
        timeout: float | None = None,
        verify_ssl: bool | None = None,
        session: requests.Session | None = None,
    ):
        self.url = (url or config.CONTROL_URL).rstrip("/")
        self.headers = headers or getattr(config, "REQUEST_HEADERS", {"Content-Type": "application/json"})
        self.timeout = timeout if timeout is not None else getattr(config, "REQUEST_TIMEOUT", 5)
        self.verify_ssl = verify_ssl if verify_ssl is not None else getattr(config, "VERIFY_SSL", True)
        self.s = session or requests.Session()

        # Repeater dùng _post_payload làm callback gửi HTTP
        self.repeater = CommandRepeater(hz=config.REPEATER_HZ, post_func=self._post_payload)

        print(f"[Control] CONTROL_URL={self.url}, timeout={self.timeout}, verify_ssl={self.verify_ssl}")

    # ---------- HTTP helper ----------
    def _post_payload(self, payload: dict):
        try:
            resp = self.s.post(
                self.url,
                json=payload,
                headers=self.headers,
                timeout=self.timeout,
                verify=self.verify_ssl,
            )
            text = ""
            try:
                # in gọn log (tránh dài)
                text = resp.text[:200]
            except Exception:
                text = "<binary>"
            print(f"[Control] POST {payload} -> {resp.status_code} {text}")
            return resp.status_code, resp.text
        except requests.exceptions.RequestException as e:
            print(f"[Control] Connection error: {e}")
            return None, str(e)

    # ---------- API công khai ----------
    def start_motion(self, command: str, **kwargs):
        """
        Bắt đầu gửi lặp lệnh (ví dụ: start_motion('move', direction='forward', speed=0.4)).
        Server sẽ nhận {"command": "move", "direction": "...", "speed": ...}
        """
        self.repeater.set(command, **kwargs)

    def stop(self):
        """
        Ngừng lặp và gửi lệnh dừng 1 lần.
        """
        self.repeater.clear()
        self._post_payload({"command": "stop"})

    def close(self):
        """
        Dừng repeater và đóng session (gọi khi thoát ứng dụng).
        """
        try:
            self.repeater.stop()
        except Exception:
            pass
        try:
            self.s.close()
        except Exception:
            pass

    # ---------- Các lệnh đặt vị trí/thái độ ----------
    def set_z(self, z: int):
        payload = {"command": "setz", "value": int(z)}
        self._post_payload(payload)

    def set_attitude(self, axis: str, value: float):
        """
        Gửi dạng chung:
          {"command": ATTITUDE_CMD, AXIS_KEY: axis, VALUE_KEY: value}
        Nếu server chưa hỗ trợ ATTITUDE_CMD, fallback về setroll/setpitch/setyaw.
        """
        payload = {
            "command": config.ATTITUDE_CMD,
            config.ATTITUDE_AXIS_KEY: axis,
            config.ATTITUDE_VALUE_KEY: float(value),
        }
        code, _ = self._post_payload(payload)
        if code is None or code >= 400:
            # fallback legacy theo từng trục
            if axis == config.AXIS_ROLL:
                self._post_payload({"command": "setroll", "value": float(value)})
            elif axis == config.AXIS_PITCH:
                self._post_payload({"command": "setpitch", "value": float(value)})
            elif axis == config.AXIS_YAW:
                self._post_payload({"command": "setyaw", "value": float(value)})

    def set_pitch(self, pitch_deg: float):
        self.set_attitude(config.AXIS_PITCH, float(pitch_deg))

    def set_roll(self, roll_deg: float):
        self.set_attitude(config.AXIS_ROLL, float(roll_deg))

    def set_yaw(self, yaw_deg: float):
        self.set_attitude(config.AXIS_YAW, float(yaw_deg))

    def set_roll_yaw(self, roll_deg: float, yaw_deg: float):
        # gửi tuần tự để giữ tương thích server hiện tại
        self.set_roll(roll_deg)
        self.set_yaw(yaw_deg)
