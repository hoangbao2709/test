# dogzilla_server/routes/status.py
# -*- coding: utf-8 -*-
from flask import Blueprint, jsonify
from ..robot import robot
from .. import config

bp = Blueprint("status", __name__)

@bp.route("/status", methods=["GET", "POST"])
def status():
    """
    Tr? JSON cho Django ROSClient.get_status():
      - ROSClient ch? l?y:
          battery = s.get("battery")
          fps     = s.get("fps")
      - C�c field kh�c ch? y?u �? debug / m? r?ng.

    V� d? response:
    {
      "robot_connected": true,
      "turn_speed_range": [-70, 70],
      "step_default": 10,
      "z_range": [75, 115],
      "z_current": 105,
      "pitch_range": [-30, 30],
      "pitch_current": 0,
      "battery": 95,
      "fw": "3.1.9",
      "fps": 30
    }
    """
    # m?c �?nh n?u �?c th?t b?i
    battery = None
    fw      = None

    if robot.dog is not None:
        try:
            if hasattr(robot.dog, "read_battery"):
                battery = robot.dog.read_battery()
        except Exception as e:
            print("[Status] read_battery error:", e)

        try:
            if hasattr(robot.dog, "read_version"):
                fw = robot.dog.read_version()
        except Exception as e:
            print("[Status] read_version error:", e)

    data = {
        "robot_connected": robot.dog is not None,
        "turn_speed_range": [
            getattr(config, "TURN_MIN", -70),
            getattr(config, "TURN_MAX",  70),
        ],
        "step_default": getattr(config, "STEP_DEFAULT", 10),
        "z_range": [
            getattr(config, "Z_MIN", 75),
            getattr(config, "Z_MAX", 115),
        ],
        "z_current": robot.z_current(),
        "pitch_range": [
            getattr(config, "PITCH_MIN", -30.0),
            getattr(config, "PITCH_MAX",  30.0),
        ],
        "pitch_current": robot.pitch_current(),
        "battery": battery,
        "fw": fw,
        "fps": getattr(config, "FRAME_FPS", 30),
    }

    return jsonify(data)
