# dogzilla_server/routes/control.py
# -*- coding: utf-8 -*-
from flask import Blueprint, request, jsonify
from ..robot import robot
from .. import config

bp = Blueprint("control", __name__)

def _ok(result: str = "ok"):
    return jsonify({"ok": True, "result": result}), 200

def _err(msg: str, code: int = 400):
    return jsonify({"ok": False, "error": msg}), code


# ====== (1) BẢNG ACTION ID – chỉnh theo tài liệu Dogzilla của bạn ======
POSTURE_ACTIONS = {
    # TODO: sửa ID theo manual của Dogzilla
    "Lie_Down": 1,
    "Stand_Up": 2,
    "Sit_Down": 3,
    "Squat": 4,
    "Crawl": 5,
}

BEHAVIOR_ACTIONS = {
    # Basic fun actions
    "Wave_Hand": 10,
    "Handshake": 11,
    "Pray": 12,
    "Stretch": 13,
    "Swing": 14,
    # Axis motion & other behaviors – gán ID tuỳ theo firmware
    "Wave_Body": 15,
    "Pee": 16,
    "Play_Ball": 17,
    "Mark_Time": 18,
    "Turn_Roll": 19,
    "Turn_Pitch": 20,
    "Turn_Yaw": 21,
    "3_Axis": 22,
    "Turn_Around": 23,
}


@bp.route("/control", methods=["POST"])
def control():
    data = request.get_json(silent=True) or {}
    cmd = (data.get("command") or "").strip().lower()
    if not cmd:
        return _err("missing 'command' field")

    # ---------- 1) Motion + stop ----------
    if cmd in ("forward", "back", "left", "right", "turnleft", "turnright", "stop"):
        step  = data.get("step")
        speed = data.get("speed")
        res = robot.do_motion(cmd, step=step, speed=speed)
        if res.startswith("error"):
            return _err(res, 500)
        return _ok(res)

    # ---------- 2) Posture ----------
    if cmd == "posture":
        name = data.get("name")
        if not name:
            return _err("posture requires 'name'")
        action_id = POSTURE_ACTIONS.get(name)
        if action_id is None:
            return _err(f"unknown posture: {name}")
        if robot.dog is None or not hasattr(robot.dog, "action"):
            return _err("robot not connected or action() unsupported", 500)
        try:
            robot.dog.action(action_id)
            return _ok(f"posture {name} -> action({action_id})")
        except Exception as e:
            return _err(str(e), 500)

    # ---------- 3) Behavior (fun + axis motion) ----------
    if cmd == "behavior":
        name = data.get("name")
        if not name:
            return _err("behavior requires 'name'")
        action_id = BEHAVIOR_ACTIONS.get(name)
        if action_id is None:
            return _err(f"unknown behavior: {name}")
        if robot.dog is None or not hasattr(robot.dog, "action"):
            return _err("robot not connected or action() unsupported", 500)
        try:
            robot.dog.action(action_id)
            return _ok(f"behavior {name} -> action({action_id})")
        except Exception as e:
            return _err(str(e), 500)

    # ---------- 4) Lidar (tạm thời chỉ log / no-op) ----------
    if cmd == "lidar":
        action = data.get("action")
        if action not in ("start", "stop"):
            return _err("lidar requires 'action' = 'start'|'stop'")
        print(f"[Control] lidar({action}) – hiện chưa nối với ROS/LiDAR thật")
        return _ok(f"lidar({action})")

    # ---------- 5) Body adjust (6 slider) ----------
    if cmd == "body_adjust":
        # Frontend gửi range [-100..100]
        tx = float(data.get("tx", 0.0))
        ty = float(data.get("ty", 0.0))
        tz = float(data.get("tz", 0.0))
        rx = float(data.get("rx", 0.0))
        ry = float(data.get("ry", 0.0))
        rz = float(data.get("rz", 0.0))

        # Map slider [-100,100] -> range cấu hình
        # Z: nội suy giữa Z_MIN..Z_MAX
        z_min = getattr(config, "Z_MIN", 75)
        z_max = getattr(config, "Z_MAX", 115)
        z_mid = 0.5 * (z_min + z_max)
        # tz = 0 => z_mid; tz = +100 => z_max; tz = -100 => z_min
        z_norm = max(-100.0, min(100.0, tz)) / 100.0
        z_target = z_mid + z_norm * (z_max - z_mid)
        robot.setz(int(z_target))

        # Roll / Pitch / Yaw: nội suy theo min/max
        roll_min = getattr(config, "ROLL_MIN", -20.0)
        roll_max = getattr(config, "ROLL_MAX",  20.0)
        pitch_min = getattr(config, "PITCH_MIN", -22.0)
        pitch_max = getattr(config, "PITCH_MAX",  22.0)
        yaw_min = getattr(config, "YAW_MIN", -16.0)
        yaw_max = getattr(config, "YAW_MAX",  16.0)

        def lerp_slider(v, lo, hi):
            v = max(-100.0, min(100.0, v)) / 100.0  # [-1,1]
            mid = 0.5 * (lo + hi)
            return mid + v * (hi - mid)

        robot.set_roll( lerp_slider(rx, roll_min,  roll_max) )
        robot.set_pitch(lerp_slider(ry, pitch_min, pitch_max))
        robot.set_yaw(  lerp_slider(rz, yaw_min,   yaw_max))

        # Tx/Ty: nếu bạn muốn dịch thân theo X/Y, có thể map thêm:
        # if robot.dog and hasattr(robot.dog, "translation"):
        #     robot.dog.translation(['x','y'], [tx_mm, ty_mm])

        return _ok("body_adjust applied")

    # ---------- 6) Legacy / status nhanh ----------
    if cmd == "status":
        return jsonify({
            "robot_connected": robot.dog is not None,
            "z_current": robot.z_current(),
            "roll_current": robot.roll_current(),
            "pitch_current": robot.pitch_current(),
            "yaw_current": robot.yaw_current(),
        })

    # ---------- unknown ----------
    return _err(f"unknown command: {cmd}")
