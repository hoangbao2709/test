# -*- coding: utf-8 -*-
import time
import platform
try:
    import ctypes
except Exception:
    ctypes = None
try:
    import Quartz  # macOS
except Exception:
    Quartz = None

from . import config

class MouseLook:
    def __init__(self, root, control):
        self.root = root
        self.control = control
        self.enabled = True
        self.center_x = None
        self.center_y = None

        self.last_cmd = None  
        self.last_speed = 0
        self.last_move_ts = 0

        self.pitch_value = float(config.PITCH_INITIAL)  
        self.last_sent_pitch = float('nan')

    def enable(self):
        self.enabled = True
        self.root.config(cursor="none")
        self._recalc_center()
        self._warp_to_center()
        print("Mouse look: ENABLED (center-locked)")

    def disable(self):
        self.enabled = False
        self.root.config(cursor="")
        print("Mouse look: DISABLED")

    def on_configure(self, _event=None):
        self._recalc_center()

    def tick(self):
        if self.enabled and self.root.winfo_exists():
            if self.center_x is None:
                self._recalc_center()
            try:
                px = self.root.winfo_pointerx()
                py = self.root.winfo_pointery()
            except Exception:
                px = py = None
            if (px is not None) and (py is not None):
                dx = px - self.center_x
                dy = py - self.center_y
                self._apply_dx_turn(dx)
                self._apply_dy_pitch(dy)
            self._warp_to_center()
        self.root.after(int(1000/config.MOUSELOOK_HZ), self.tick)

    # ---- internals ----
    def _recalc_center(self):
        self.root.update_idletasks()
        self.center_x = self.root.winfo_rootx() + self.root.winfo_width() // 2
        self.center_y = self.root.winfo_rooty() + self.root.winfo_height() // 2

    def _warp_to_center(self):
        # Try Tk warp first
        try:
            self.root.event_generate("<Motion>", warp=True,
                                     x=self.root.winfo_width()//2,
                                     y=self.root.winfo_height()//2)
            return True
        except Exception:
            pass
        # OS-level fallback
        sysname = platform.system()
        try:
            if sysname == "Windows" and ctypes is not None:
                ctypes.windll.user32.SetCursorPos(self.center_x, self.center_y)
                return True
            if sysname == "Darwin" and Quartz is not None:
                Quartz.CGWarpMouseCursorPosition((self.center_x, self.center_y))
                Quartz.CGAssociateMouseAndMouseCursorPosition(True)
                return True
        except Exception:
            pass
        return False

    def _apply_dx_turn(self, dx: int):
        now = int(time.time() * 1000)
        absdx = abs(dx)
        if absdx < config.DEADZONE:
            if self.last_cmd in ("turnleft", "turnright") and (now - self.last_move_ts) < config.HOLD_MS:
                return
            if self.last_cmd != "stop":
                self.control.stop()
                self.last_cmd = "stop"
                self.last_speed = 0
            return
        self.last_move_ts = now
        norm = min(1.0, absdx / float(config.SCALE_PX))
        curved = norm ** config.SPEED_GAMMA
        target = config.TURN_SPEED_MIN + (config.TURN_SPEED_MAX - config.TURN_SPEED_MIN) * curved
        if self.last_speed == 0:
            smoothed = target
        else:
            a = config.SPEED_SMOOTH_ALPHA
            smoothed = a * target + (1 - a) * self.last_speed
        speed_int = int(round(min(config.TURN_SPEED_MAX, max(config.TURN_SPEED_MIN, smoothed))))
        direction = "turnleft" if dx < 0 else "turnright"
        if direction == self.last_cmd and abs(speed_int - self.last_speed) < config.SPEED_UPDATE_DELTA:
            return
        self.control.start_motion(direction, speed=speed_int)
        self.last_cmd = direction
        self.last_speed = speed_int

    def _apply_dy_pitch(self, dy: int):
        absdy = abs(dy)
        if absdy < config.PITCH_DEADZONE_PIX:
            return 
        norm = min(1.0, absdy / float(config.PITCH_SCALE_PY))
        curved = norm ** config.PITCH_GAMMA
        step = curved * config.PITCH_MAX_STEP_DEG
        delta = (-step) if dy < 0 else (+step)  
        new_pitch = self.pitch_value + delta
        a = config.PITCH_SMOOTH_ALPHA
        smoothed = a * new_pitch + (1 - a) * self.pitch_value
        smoothed = max(config.PITCH_MIN, min(config.PITCH_MAX, smoothed))
        if (self.last_sent_pitch != self.last_sent_pitch) or (abs(smoothed - self.last_sent_pitch) >= config.PITCH_UPDATE_DELTA):
            self.control.set_pitch(int(round(smoothed)))
            self.last_sent_pitch = smoothed
        self.pitch_value = smoothed