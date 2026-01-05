# -*- coding: utf-8 -*-
from flask import Flask, jsonify, request
import atexit
from . import config, create_app
from . import config
from .camera import init_camera, cleanup_camera
from .routes.control import bp as control_bp
from .routes.status import bp as status_bp
from .routes.camera import bp as camera_bp


app = Flask(__name__)

# Log m?i request d? b?n th?y r� GET /camera, GET /frame...
@app.before_request
def _log_request():
    try:
        print(f"{request.remote_addr} {request.method} {request.path}")
    except Exception:
        pass

# �ang k� route
app.register_blueprint(control_bp)
app.register_blueprint(status_bp)
app.register_blueprint(camera_bp)

# Root
@app.route("/")
def root():
    return jsonify({
        "status": "ok",
        "endpoints": {
            "control": "POST /control",
            "status": "GET /status",
            "camera": "GET /camera (MJPEG)",
            "frame": "GET /frame (single JPEG)"
        }
    })

# Trang test don gi?n
@app.route("/test")
def test_page():
    return """
<!doctype html>
<html>
  <body>
    <h3>/frame (single JPEG)</h3>
    <img src="/frame" width="640" height="480"/>
    <h3>/camera (MJPEG)</h3>
    <img src="/camera" width="640" height="480"/>
  </body>
</html>
"""


@app.route("/health")
def health():
    return jsonify({"ok": True})


# Kh?i t?o / d?n d?p camera
init_camera()
atexit.register(cleanup_camera)

if __name__ == "__main__":
    print(f"[Server] HTTP_PORT={config.HTTP_PORT}  CAMERA_INDEX={config.CAMERA_INDEX}  "
          f"DOG={config.DOG_PORT}@{config.DOG_BAUD}")
    app.run(host="0.0.0.0", port=config.HTTP_PORT, threaded=True)
