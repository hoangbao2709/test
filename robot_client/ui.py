# -*- coding: utf-8 -*-
import time
import cv2
from PIL import Image, ImageTk
import tkinter as tk
from . import config

class AppUI:
    def __init__(self, root, control, camera, mouselook):
        self.root = root
        self.control = control
        self.camera = camera
        self.mouselook = mouselook

        self.ui_ready = False
        self.pressed_keys = set()
        self.ctrl_down = False

        # Track current Z locally (absolute)
        self.z_value = config.Z_INITIAL

        # === NEW: E/Q hold state for roll & yaw ===
        self.eq_dir = 0             # +1 when E, -1 when Q, 0 when none
        self.roll_value = 0.0       # current roll setpoint
        self.yaw_value  = 0.0       # current yaw  setpoint
        self._last_eq_ts = None
        self._last_sent_roll = float('nan')
        self._last_sent_yaw  = float('nan')

        self._build()
        self._bind()

    def _build(self):
        self.root.title("Dogzilla Control Panel (PUBG-style)")
        self.root.geometry("900x720")

        self.lbl_img = tk.Label(self.root)
        self.lbl_img.pack(pady=10)

        pad = 10
        panel = tk.Frame(self.root)
        panel.pack(pady=10)
        self.panel = panel

        def make_hover_button(parent, text, command_name, width=14, with_speed=False):
            btn = tk.Button(parent, text=text, width=width)
            def go(_=None):
                if self.ui_ready:
                    if with_speed:
                        self.control.start_motion(command_name, speed=config.TURN_SPEED)
                    else:
                        self.control.start_motion(command_name)
            btn.bind("<Enter>",           go)
            btn.bind("<Leave>",           lambda e: self.control.stop())
            btn.bind("<ButtonPress-1>",   go)
            btn.bind("<ButtonRelease-1>", lambda e: self.control.stop())
            return btn

        grid = tk.Frame(panel)
        grid.pack()
        pad = 10
        btn_w = make_hover_button(grid, "W (Forward)", "forward")
        btn_a = make_hover_button(grid, "A (Left)",    "left")
        btn_s = make_hover_button(grid, "S (Back)",    "back")
        btn_d = make_hover_button(grid, "D (Right)",   "right")

        tk.Label(grid, text="").grid(row=0, column=0, padx=pad, pady=pad)
        btn_w.grid(row=0, column=1, padx=pad, pady=pad)
        tk.Label(grid, text="").grid(row=0, column=2, padx=pad, pady=pad)
        btn_a.grid(row=1, column=0, padx=pad, pady=pad)
        tk.Label(grid, text="").grid(row=1, column=1, padx=pad, pady=pad)
        btn_d.grid(row=1, column=2, padx=pad, pady=pad)
        tk.Label(grid, text="").grid(row=2, column=0, padx=pad, pady=pad)
        btn_s.grid(row=2, column=1, padx=pad, pady=pad)
        tk.Label(grid, text="").grid(row=2, column=2, padx=pad, pady=pad)

        btn_turn_l = make_hover_button(panel, "Turn Left",  "turnleft",  with_speed=True)
        btn_turn_r = make_hover_button(panel, "Turn Right", "turnright", with_speed=True)
        btn_turn_l.pack(side="left", padx=pad)
        btn_turn_r.pack(side="left", padx=pad)

        tk.Button(panel, text="STOP", width=14, command=self.control.stop).pack(pady=(pad, 0))

        # Optional info labels
        self.lbl_z    = tk.Label(self.root, text=f"Z: {self.z_value}")
        self.lbl_roll = tk.Label(self.root, text=f"Roll: {int(round(self.roll_value))}°")
        self.lbl_yaw  = tk.Label(self.root, text=f"Yaw: {int(round(self.yaw_value))}°")
        self.lbl_z.pack()
        self.lbl_roll.pack()
        self.lbl_yaw.pack(pady=(0, 10))

    def _bind(self):
        self.root.bind("<KeyPress>",   self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.bind("<Configure>",  self.mouselook.on_configure)
        self.root.bind_all("<ButtonRelease-1>", lambda e: self.control.stop())

        # Mouse wheel -> adjust Z (Windows/macOS)
        self.root.bind("<MouseWheel>", self._on_wheel)
        self.lbl_img.bind("<MouseWheel>", self._on_wheel)
        # Linux (X11): Button-4 up, Button-5 down
        self.root.bind("<Button-4>", lambda e: self._on_wheel_linux(1))
        self.root.bind("<Button-5>", lambda e: self._on_wheel_linux(-1))
        self.lbl_img.bind("<Button-4>", lambda e: self._on_wheel_linux(1))
        self.lbl_img.bind("<Button-5>", lambda e: self._on_wheel_linux(-1))

        # timers
        self.root.after(100, self._ui_update)
        self.root.after(700,  self._enable_ui)
        self.root.after(1000, self.mouselook.enable)
        self.root.after(int(1000/config.MOUSELOOK_HZ), self.mouselook.tick)
        # NEW: E/Q hold tick
        self.root.after(int(1000/config.ATT_HOLD_HZ), self._eq_tick)

    # ---- wheel handlers (Z) ----
    def _on_wheel(self, event):
        direction = 1 if event.delta > 0 else -1 if event.delta < 0 else 0
        if direction:
            self._change_z(direction)

    def _on_wheel_linux(self, direction: int):
        self._change_z(direction)

    def _change_z(self, direction: int):
        step = config.SCROLL_Z_STEP * (1 if direction >= 0 else -1)
        new_z = max(config.Z_MIN, min(config.Z_MAX, self.z_value + step))
        if new_z != self.z_value:
            self.z_value = new_z
            self.lbl_z.configure(text=f"Z: {self.z_value}")
            self.control.set_z(self.z_value)

    # ---- key handlers ----
    def _enable_ui(self):
        self.ui_ready = True
        print("UI is now ready (hover enabled).")

    def _on_key_press(self, event):
        key = event.keysym
        if key in ("Control_L", "Control_R"):
            if not self.ctrl_down:
                self.ctrl_down = True
                if self.mouselook.enabled:
                    self.mouselook.disable()
                    self.control.stop()
                else:
                    self.mouselook.enable()
            return
        if not self.ui_ready:
            return

        k = key.lower()
        if k not in self.pressed_keys:
            self.pressed_keys.add(k)
            if k == "w":
                self.control.start_motion("forward")
            elif k == "s":
                self.control.start_motion("back")
            elif k == "a":
                self.control.start_motion("left")
            elif k == "d":
                self.control.start_motion("right")
            elif k == "q":
                self.eq_dir = +1   # increase roll & yaw
            elif k == "e":
                self.eq_dir = -1   # decrease roll & yaw

    def _on_key_release(self, event):
        key = event.keysym
        if key in ("Control_L", "Control_R"):
            self.ctrl_down = False
            return

        k = key.lower()
        if k in self.pressed_keys:
            self.pressed_keys.remove(k)
            if k in ("w", "a", "s", "d"):
                self.control.stop()

        # When releasing E or Q: snap roll & yaw back to 0 immediately
        if k in ("e", "q"):
            self.eq_dir = 0
            self.roll_value = 0.0
            self.yaw_value  = 0.0
            self._send_roll_yaw_if_needed(force=True)
            self._update_attitude_labels()

    # ---- E/Q hold tick ----
    def _eq_tick(self):
        # Periodic update while holding E/Q
        now = time.time()
        if self._last_eq_ts is None:
            self._last_eq_ts = now
        dt = now - self._last_eq_ts
        self._last_eq_ts = now

        if self.eq_dir != 0:
            delta = self.eq_dir * config.ROLL_YAW_RATE_DPS * dt
            # integrate and clamp
            self.roll_value = max(config.ROLL_MIN, min(config.ROLL_MAX, self.roll_value + delta))
            self.yaw_value  = max(config.YAW_MIN,  min(config.YAW_MAX,  self.yaw_value  + delta))
            self._send_roll_yaw_if_needed()
            self._update_attitude_labels()

        # schedule next tick
        self.root.after(int(1000/config.ATT_HOLD_HZ), self._eq_tick)

    def _send_roll_yaw_if_needed(self, force: bool = False):
        need_roll = force or (self._last_sent_roll != self._last_sent_roll) or (abs(self.roll_value - self._last_sent_roll) >= config.ATT_UPDATE_DELTA)
        need_yaw  = force or (self._last_sent_yaw  != self._last_sent_yaw ) or (abs(self.yaw_value  - self._last_sent_yaw ) >= config.ATT_UPDATE_DELTA)
        if need_roll or need_yaw:
            # send current values
            if need_roll:
                self.control.set_roll(self.roll_value)
                self._last_sent_roll = self.roll_value
            if need_yaw:
                self.control.set_yaw(self.yaw_value)
                self._last_sent_yaw = self.yaw_value

    def _update_attitude_labels(self):
        self.lbl_roll.configure(text=f"Roll: {int(round(self.roll_value))}°")
        self.lbl_yaw.configure(text=f"Yaw: {int(round(self.yaw_value))}°")

    # ---- video UI update ----
    def _ui_update(self):
        frame = self.camera.get_latest()
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            from PIL import Image
            img = Image.fromarray(rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.lbl_img.imgtk = imgtk
            self.lbl_img.configure(image=imgtk)
        # update labels periodically anyway
        self._update_attitude_labels()
        self.root.after(config.UI_FPS_MS, self._ui_update)
