# -*- coding: utf-8 -*-
import tkinter as tk

from .control import Control
from .camera import Camera
from .mouselook import MouseLook
from .ui import AppUI
from . import config


def main():
    root = tk.Tk()
    root.title("Dogzilla Client")

    # ---- Control ----
    control = Control(
        url=config.CONTROL_URL,
        headers=config.REQUEST_HEADERS,
        timeout=config.REQUEST_TIMEOUT,
        verify_ssl=config.VERIFY_SSL,
    )

    # ---- Camera ----
    camera = Camera(
        url=config.CAMERA_URL,
        headers=config.REQUEST_HEADERS,
        timeout=config.REQUEST_TIMEOUT,
        verify_ssl=config.VERIFY_SSL,
    )
    camera.start()

    # ---- MouseLook + UI ----
    mouselook = MouseLook(root, control)
    app = AppUI(root, control, camera, mouselook)

    # dừng control (reset) khi khởi động
    try:
        control.stop()
    except Exception as e:
        print(f"[WARN] control.stop() at start failed: {e}")

    # ---- Clean up khi thoát ----
    def on_closing():
        try:
            control.stop()
        except Exception:
            pass
        try:
            camera.stop()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    print(f"[INFO] Dogzilla UI starting...")
    print(f"[INFO] CONTROL_URL = {config.CONTROL_URL}")
    print(f"[INFO] CAMERA_URL  = {config.CAMERA_URL}")
    if config.API_KEY:
        print(f"[INFO] Using API key auth")
    if not config.VERIFY_SSL and config.BASE_URL.startswith("https://"):
        print(f"[WARN] SSL verify disabled!")

    root.mainloop()


if __name__ == "__main__":
    main()
