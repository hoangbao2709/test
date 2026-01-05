# -*- coding: utf-8 -*-
import os

# HTTP
HTTP_PORT     = int(os.environ.get("HTTP_PORT", "9000"))

# Camera (server side)
CAMERA_INDEX  = int(os.environ.get("CAMERA_INDEX", "0"))
FRAME_W       = int(os.environ.get("CAM_WIDTH", "640"))
FRAME_H       = int(os.environ.get("CAM_HEIGHT", "480"))
FRAME_FPS     = int(os.environ.get("CAM_FPS", "30"))

# Robot serial
DOG_PORT      = os.environ.get("DOGZILLA_PORT", "/dev/ttyAMA0")
DOG_BAUD      = int(os.environ.get("DOGZILLA_BAUD", "115200"))

# Motion defaults
STEP_DEFAULT  = int(os.environ.get("STEP_DEFAULT", "8"))
TURN_MIN      = int(os.environ.get("TURN_MIN", "-70"))
TURN_MAX      = int(os.environ.get("TURN_MAX", "70"))

# Z height
Z_MIN         = int(os.environ.get("Z_MIN", "75"))
Z_MAX         = int(os.environ.get("Z_MAX", "110"))
Z_DEFAULT     = int(os.environ.get("Z_DEFAULT", "105"))

# Attitude ranges (deg)
ROLL_MIN      = float(os.environ.get("ROLL_MIN", "-20"))
ROLL_MAX      = float(os.environ.get("ROLL_MAX", "20"))
YAW_MIN       = float(os.environ.get("YAW_MIN",  "-11"))
YAW_MAX       = float(os.environ.get("YAW_MAX",   "11"))
PITCH_MIN     = float(os.environ.get("PITCH_MIN", "-30"))
PITCH_MAX     = float(os.environ.get("PITCH_MAX", "30"))

# Defaults (deg)
ROLL_DEFAULT  = float(os.environ.get("ROLL_DEFAULT", "0"))
YAW_DEFAULT   = float(os.environ.get("YAW_DEFAULT",  "0"))
PITCH_DEFAULT = float(os.environ.get("PITCH_DEFAULT","0"))
