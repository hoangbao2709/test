# -*- coding: utf-8 -*-
from typing import Optional
import threading
from . import config


from DOGZILLALib import DOGZILLA as _DOG



class Robot:
    """DOGZILLA wrapper: motion + Z + Attitude (roll/pitch/yaw) với clamp & state phía server."""

    def __init__(self) -> None:
        self.dog = None
        # locks
        self._z_lock = threading.Lock()
        self._att_lock = threading.Lock()
        # server-side state
        self._current_z     = int(config.Z_DEFAULT)
        self._roll_current  = float(config.ROLL_DEFAULT)
        self._pitch_current = float(config.PITCH_DEFAULT)
        self._yaw_current   = float(config.YAW_DEFAULT)

        if _DOG is not None:
            try:
                self.dog = _DOG(port=config.DOG_PORT, baud=config.DOG_BAUD, verbose=False)
                print(f"[DOGZILLA] Connected on {config.DOG_PORT} @ {config.DOG_BAUD}")
            except Exception as e:
                print("[DOGZILLA] Init error:", e)
                self.dog = None
        else:
            print("[DOGZILLA] Library not found. Running without robot.")

    # ---------- utils ----------
    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return lo if v < lo else hi if v > hi else v

    # ---------- motion ----------
    def resolve_value(self, *, step: Optional[int], speed: Optional[int], is_turn: bool) -> int:
        if speed is not None:
            val = int(speed)
            return int(self._clamp(val, config.TURN_MIN, config.TURN_MAX)) if is_turn else val
        if step is not None:
            return int(step)
        return config.STEP_DEFAULT

    def do_motion(self, cmd: str, *, step: Optional[int] = None, speed: Optional[int] = None) -> str:
        if self.dog is None:
            return "robot not connected"

        is_turn = cmd in ("turnleft", "turnright")
        val = self.resolve_value(step=step, speed=speed, is_turn=is_turn)
        try:
            if cmd == "forward":     self.dog.forward(int(val))
            elif cmd == "back":      self.dog.back(int(val))
            elif cmd == "left":      self.dog.left(int(val))
            elif cmd == "right":     self.dog.right(int(val))
            elif cmd == "turnleft":  self.dog.turnleft(int(val))
            elif cmd == "turnright": self.dog.turnright(int(val))
            elif cmd == "stop":      self.dog.stop()
            else:
                return f"unknown command: {cmd}"
        except Exception as e:
            return f"error: {e}"

        if cmd == "stop":
            return "ok: stop"
        if is_turn:
            return f"ok: {cmd}(speed={val})"
        return f"ok: {cmd}({val})"

    # ---------- Z ----------
    def setz(self, z: int) -> str:
        z = int(self._clamp(int(z), config.Z_MIN, config.Z_MAX))
        if self.dog is None:
            with self._z_lock:
                self._current_z = z
            return f"ok: setz({z}) (robot not connected)"
        try:
            if hasattr(self.dog, 'translation'):
                self.dog.translation('z', int(z))
            elif hasattr(self.dog, 'setz'):
                self.dog.setz(int(z))
            else:
                return "error: setz unsupported by DOGZILLA lib"
            with self._z_lock:
                self._current_z = z
            return f"ok: setz({z})"
        except Exception as e:
            return f"error: {e}"

    def adjustz(self, delta: int) -> str:
        with self._z_lock:
            target = self._current_z + int(delta)
        return self.setz(target)

    def z_current(self) -> int:
        with self._z_lock:
            return self._current_z

    # ---------- Attitude ----------
    def _set_axis(self, ax: str, val: float) -> str:
        """Gọi đúng chữ ký: dog.attitude(axis, value) (2 đối số sau self)."""
        if self.dog is None:
            with self._att_lock:
                if ax == 'r':
                    self._roll_current = val
                elif ax == 'p':
                    self._pitch_current = val
                elif ax == 'y':
                    self._yaw_current = val
            return f"ok: attitude({ax}={val}) (robot not connected)"

        try:
            if hasattr(self.dog, "attitude"):
                self.dog.attitude(ax, int(val))  # CHỈ 2 tham số; cast int cho chắc
            elif ax == 'r' and hasattr(self.dog, "setroll"):
                self.dog.setroll(int(val))
            elif ax == 'p' and hasattr(self.dog, "setpitch"):
                self.dog.setpitch(int(val))
            elif ax == 'y' and hasattr(self.dog, "setyaw"):
                self.dog.setyaw(int(val))
            else:
                return "error: attitude unsupported by DOGZILLA lib"

            with self._att_lock:
                if ax == 'r':
                    self._roll_current = val
                elif ax == 'p':
                    self._pitch_current = val
                else:
                    self._yaw_current = val
            return f"ok: attitude({ax}={val})"
        except Exception as e:
            return f"error: {e}"

    def set_attitude(self, axis: str, value: float) -> str:
        ax = str(axis).lower()[:1]
        if ax not in ('r', 'p', 'y'):
            return "error: invalid axis"
        if ax == 'r':
            v = self._clamp(float(value), config.ROLL_MIN,  config.ROLL_MAX)
        elif ax == 'p':
            v = self._clamp(float(value), config.PITCH_MIN, config.PITCH_MAX)
        else:
            v = self._clamp(float(value), config.YAW_MIN,   config.YAW_MAX)
        return self._set_axis(ax, v)

    # convenience
    def set_roll(self, v: float)  -> str:
        return self.set_attitude('r', v)

    def set_pitch(self, v: float) -> str:
        return self.set_attitude('p', v)

    def set_yaw(self, v: float)   -> str:
        return self.set_attitude('y', v)

    # status readers
    def roll_current(self) -> float:
        with self._att_lock:
            return self._roll_current

    def pitch_current(self) -> float:
        with self._att_lock:
            return self._pitch_current

    def yaw_current(self) -> float:
        with self._att_lock:
            return self._yaw_current


# Global singleton
robot = Robot()
