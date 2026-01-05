#!/usr/bin/env python3
import os
import math
import threading
import json
from http.server import SimpleHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import cgi
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow

from path_planner import plan_path  # A* + simplify


def quat_to_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2 * (w * z + x * y),
        1 - 2 * (y * y + z * z)
    )


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PNG_PATH = os.path.join(BASE_DIR, "map.png")

# Thu muc luu map PGM + YAML
MAP_SAVE_DIR = os.path.join(BASE_DIR, "saved_maps")
os.makedirs(MAP_SAVE_DIR, exist_ok=True)

GOAL_REQUEST = None
CLEAR_REQUEST = False
SAVE_REQUEST = None       # ten map can save
LOAD_REQUEST = None       # duong dan yaml can load
LOCK = threading.Lock()

# Pose hien tai cua robot trong frame "map"
CURRENT_POSE = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,  # rad
}


def set_goal_request(u, v):
    global GOAL_REQUEST
    with LOCK:
        GOAL_REQUEST = (u, v)


def pop_goal_request():
    global GOAL_REQUEST
    with LOCK:
        req = GOAL_REQUEST
        GOAL_REQUEST = None
    return req


def request_clear_path():
    global CLEAR_REQUEST
    with LOCK:
        CLEAR_REQUEST = True


def pop_clear_request():
    global CLEAR_REQUEST
    with LOCK:
        flag = CLEAR_REQUEST
        CLEAR_REQUEST = False
    return flag


def set_save_request(name: str):
    global SAVE_REQUEST
    with LOCK:
        SAVE_REQUEST = name


def pop_save_request():
    global SAVE_REQUEST
    with LOCK:
        name = SAVE_REQUEST
        SAVE_REQUEST = None
    return name


def set_load_request(yaml_path: str):
    global LOAD_REQUEST
    with LOCK:
        LOAD_REQUEST = yaml_path


def pop_load_request():
    global LOAD_REQUEST
    with LOCK:
        path = LOAD_REQUEST
        LOAD_REQUEST = None
    return path


class ImageServer(SimpleHTTPRequestHandler):
    # GET
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # ============== HTML chinh ==============
        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            html = """
<html>
<head>
  <meta charset="utf-8"/>
  <style>
    html, body {
      margin: 0;
      padding: 0;
      background: #000;
      color: #fff;
      font-family: Arial, sans-serif;
      height: 100%;
      overflow: hidden;
    }
    .wrap {
      position: relative;
      width: 100%;
      height: 100%;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    #m {
      display: block;
      max-width: 100%;
      max-height: 100%;
      border: 2px solid #444;
      cursor: crosshair;
    }
    .toolbar {
      position: absolute;
      top: 10px;
      left: 10px;
      z-index: 10;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .btn {
      padding: 8px 14px;
      font-size: 14px;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-weight: 600;
    }
    .btn-danger {
      background: #e11d48;
      color: #fff;
    }
    .btn-secondary {
      background: #0284c7;
      color: #fff;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="toolbar">
      <button class="btn btn-danger"
        onclick="fetch('/clear_path').catch(()=>{});">
        STOP & CLEAR PATH
      </button>
      <button class="btn btn-secondary"
        onclick="saveMapPrompt();">
        SAVE MAP
      </button>
      <button class="btn btn-secondary"
        onclick="window.open('/upload_map_form','_blank');">
        LOAD MAP (UPLOAD)
      </button>
    </div>
    <img id="m" src="map.png">
  </div>

  <script>
    const img = document.getElementById("m");
    // auto reload map
    setInterval(() => {
      img.src = "map.png?" + Date.now();
    }, 500);

    // click chon goal
    img.addEventListener("click", (e) => {
      const rect = img.getBoundingClientRect();
      const ux = (e.clientX - rect.left) / rect.width;
      const uy_screen = (e.clientY - rect.top) / rect.height;
      const v = 1.0 - uy_screen;
      fetch(`/set_goal?u=${ux}&v=${v}`).catch(() => {});
    });

    function saveMapPrompt() {
      const name = prompt("Nhap ten map de luu (vd: phong_ngu):");
      if (!name) return;
      const safe = name.trim();
      if (!safe) return;
      fetch(`/save_map?name=${encodeURIComponent(safe)}`).catch(()=>{});
    }
  </script>
</body>
</html>
"""
            self.wfile.write(html.encode("utf-8"))
            return

        # ============== anh map ==============
        if path.startswith("/map.png"):
            try:
                with open(MAP_PNG_PATH, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-type", "image/png")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception:
                self.send_error(404, "map.png not found")
            return

        # ============== API pose ==============
        if path == "/pose":
            body = json.dumps(CURRENT_POSE).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # ============== Set goal tu web ==============
        if path.startswith("/set_goal"):
            try:
                qs = parse_qs(parsed.query)
                u = float(qs.get("u", [None])[0])
                v = float(qs.get("v", [None])[0])
                if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
                    raise ValueError("u,v out of range")
                set_goal_request(u, v)
                self.send_response(200)
                self.send_header("Content-type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write("OK".encode("utf-8"))
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return

        # ============== STOP & CLEAR PATH ==============
        if path.startswith("/clear_path"):
            request_clear_path()
            self.send_response(200)
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write("CLEARED".encode("utf-8"))
            return

        # ============== SAVE MAP ==============
        if path.startswith("/save_map"):
            try:
                qs = parse_qs(parsed.query)
                name = qs.get("name", [None])[0]
                if not name:
                    raise ValueError("missing name")
                safe = "".join(c for c in name if c.isalnum() or c in "-_")
                if not safe:
                    raise ValueError("bad name")
                set_save_request(safe)
                self.send_response(200)
                self.send_header("Content-type", "text/html; charset=utf-8")
                self.end_headers()
                msg = f"""
<html><body style="background:#000;color:#fff;font-family:Arial">
  <p>Da nhan yeu cau luu map: <b>{safe}</b>.</p>
  <p>File se luu trong thu muc saved_maps:</p>
  <p><code>/maps/{safe}.yaml</code> va <code>/maps/{safe}.pgm</code></p>
  <a href="/">Quay lai map</a>
</body></html>
"""
                self.wfile.write(msg.encode("utf-8"))
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return

        # ============== PHUC VU FILE YAML/PGM DA LUU ==============
        if path.startswith("/maps/"):
            rel = path[len("/maps/") :]
            fname = os.path.basename(rel)
            fpath = os.path.join(MAP_SAVE_DIR, fname)
            if not os.path.isfile(fpath):
                self.send_error(404, "map file not found")
                return

            if fname.endswith(".yaml") or fname.endswith(".yml"):
                ctype = "text/yaml; charset=utf-8"
                mode = "r"
                is_bin = False
            else:
                ctype = "image/x-portable-graymap"
                mode = "rb"
                is_bin = True

            try:
                self.send_response(200)
                self.send_header("Content-type", ctype)
                self.end_headers()
                with open(fpath, mode) as f:
                    data = f.read()
                if is_bin:
                    self.wfile.write(data)
                else:
                    self.wfile.write(data.encode("utf-8"))
            except Exception:
                self.send_error(500, "error reading map file")
            return

        # ============== FORM UPLOAD MAP ==============
        if path == "/upload_map_form":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            html = """
<html><body style="background:#000;color:#fff;font-family:Arial">
  <h2>Upload MAP (YAML + PGM)</h2>
  <form action="/upload_map" method="POST" enctype="multipart/form-data">
    YAML file: <input type="file" name="yaml"><br><br>
    PGM file:  <input type="file" name="pgm"><br><br>
    <button type="submit">Upload & Load</button>
  </form>
  <p>Map sau khi load se duoc dung de ve va A*.</p>
</body></html>
"""
            self.wfile.write(html.encode("utf-8"))
            return

        # ============== Mac dinh 404 ==============
        self.send_error(404)

    # POST: upload map
    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != "/upload_map":
            self.send_error(404)
            return

        ctype = self.headers.get("content-type")
        if not ctype or not ctype.startswith("multipart/form-data"):
            self.send_error(400, "Not multipart/form-data")
            return

        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={"REQUEST_METHOD": "POST", "CONTENT_TYPE": ctype},
        )

        yaml_file = form.getfirst("yaml")
        yaml_item = form["yaml"] if "yaml" in form else None
        pgm_item = form["pgm"] if "pgm" in form else None

        if not yaml_item or not pgm_item or not yaml_item.filename or not pgm_item.filename:
            self.send_error(400, "YAML/PGM missing")
            return

        yaml_name = os.path.basename(yaml_item.filename)
        pgm_name = os.path.basename(pgm_item.filename)

        yaml_path = os.path.join(MAP_SAVE_DIR, yaml_name)
        pgm_path = os.path.join(MAP_SAVE_DIR, pgm_name)

        with open(yaml_path, "wb") as f:
            f.write(yaml_item.file.read())
        with open(pgm_path, "wb") as f:
            f.write(pgm_item.file.read())

        # thong bao cho node ROS load map nay
        set_load_request(yaml_path)

        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.end_headers()
        html = f"""
<html><body style="background:#000;color:#fff;font-family:Arial">
  <h3>Upload OK</h3>
  <p>YAML: {yaml_name}</p>
  <p>PGM: {pgm_name}</p>
  <p>Node se load map nay neu khong loi.</p>
  <a href="/">Quay lai map</a>
</body></html>
"""
        self.wfile.write(html.encode("utf-8"))


class LiveMapWeb(Node):
    def __init__(self):
        super().__init__("slam_live_map_viewer")

        self.map_topic = os.environ.get("MAP_TOPIC", "/map")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")

        self.map = None
        self.map_msg = None
        self.map_frame = "map"
        self.grid = None
        self.path_xy = None

        os.makedirs(BASE_DIR, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.cb_map,
            qos,
        )

        self.path_pub = self.create_publisher(Path, "/a_star_path", 1)
        self.goal_pub = self.create_publisher(PoseStamped, "/a_star_goal", 1)

        self.get_logger().info(
            f"[slam_live_map] HTTP: http://<IP_RPI>:8080/ | subscribe {self.map_topic}"
        )

        self.timer = self.create_timer(0.5, self.update_image)

    def cb_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_frame = msg.header.frame_id or "map"

        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # Grid 0=free, 1=obstacle/unknown
        grid = np.ones((h, w), dtype=np.uint8)
        grid[data == 0] = 0
        grid[data < 0] = 0
        grid[data > 50] = 1
        self.grid = grid

        img = np.zeros((h, w, 3), dtype=np.float32)
        img[:] = [0.5, 0.5, 0.5]
        img[data == 0] = [1.0, 1.0, 1.0]
        img[data > 50] = [0.0, 0.0, 0.0]
        self.map = np.flipud(img)

    # ===== xu ly request tu web =====

    def process_goal_request_if_any(self):
        req = pop_goal_request()
        if req is None or self.map_msg is None or self.grid is None or self.map is None:
            return

        u, v = req
        info = self.map_msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        h_img, w_img, _ = self.map.shape
        c_img = int(u * (w_img - 1))
        r_img = int((1.0 - v) * (h_img - 1))
        c_img = max(0, min(c_img, w_img - 1))
        r_img = max(0, min(r_img, h_img - 1))

        r_grid = h_img - 1 - r_img
        c_grid = c_img

        h_grid, w_grid = self.grid.shape
        r_grid = max(0, min(r_grid, h_grid - 1))
        c_grid = max(0, min(c_grid, w_grid - 1))

        gx = ox + (c_grid + 0.5) * res
        gy = oy + (r_grid + 0.5) * res

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(
                f"Khong lay duoc TF map->{self.base_frame}: {e}"
            )
            return

        self.get_logger().info(
            f"Click u={u:.3f}, v={v:.3f} -> cell=({r_grid},{c_grid}), "
            f"goal_world=({gx:.2f},{gy:.2f})"
        )

        self.path_xy = plan_path(
            self.grid,
            info,
            (rx, ry),
            (gx, gy),
            logger=self.get_logger(),
        )

        if self.path_xy and len(self.path_xy) >= 2:
            self.publish_path_and_goal(self.path_xy, self.map_frame)

    def process_clear_request_if_any(self):
        if not pop_clear_request():
            return

        self.path_xy = None

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        self.path_pub.publish(path_msg)

        self.get_logger().info("Clear path request: published empty /a_star_path.")

    def process_save_request_if_any(self):
        name = pop_save_request()
        if name is None:
            return
        if self.map_msg is None:
            self.get_logger().warn("SAVE_MAP: map_msg is None, cannot save.")
            return

        info = self.map_msg.info
        h = info.height
        w = info.width
        data = np.array(self.map_msg.data, dtype=np.int16).reshape(h, w)

        # Gray image for map_server:
        # -1 unknown -> 205, 0 free -> 254, >50 occ -> 0
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data > 50] = 0

        pgm_path = os.path.join(MAP_SAVE_DIR, name + ".pgm")
        yaml_path = os.path.join(MAP_SAVE_DIR, name + ".yaml")

        # Write PGM (no flip, theo layout OccupancyGrid)
        with open(pgm_path, "wb") as f:
            header = f"P5\n{w} {h}\n255\n"
            f.write(header.encode("ascii"))
            f.write(img.tobytes(order="C"))

        ox = info.origin.position.x
        oy = info.origin.position.y
        oz = info.origin.position.z

        with open(yaml_path, "w") as f:
            f.write(f"image: {os.path.basename(pgm_path)}\n")
            f.write(f"resolution: {info.resolution}\n")
            f.write(f"origin: [{ox}, {oy}, {oz}]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")

        self.get_logger().info(
            f"SAVE_MAP: Saved to {yaml_path} (PGM: {pgm_path})"
        )

    def _load_pgm(self, pgm_path: str):
        with open(pgm_path, "rb") as f:
            magic = f.readline().strip()
            if magic != b"P5":
                raise RuntimeError("Not P5 PGM")
            # skip comments
            def _read_non_comment():
                line = f.readline()
                while line.startswith(b"#"):
                    line = f.readline()
                return line

            line = _read_non_comment()
            parts = line.split()
            while len(parts) < 2:
                # width height may be on multiple lines
                line2 = _read_non_comment()
                parts += line2.split()
            w, h = int(parts[0]), int(parts[1])

            maxval = int(_read_non_comment())
            if maxval > 255:
                raise RuntimeError("Only 8-bit PGM supported")

            data = f.read(w * h)
            if len(data) != w * h:
                raise RuntimeError("PGM size mismatch")
            arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        return arr

    def process_load_request_if_any(self):
        yaml_path = pop_load_request()
        if yaml_path is None:
            return

        try:
            with open(yaml_path, "r") as f:
                meta = yaml.safe_load(f)
            img_name = meta["image"]
            res = float(meta["resolution"])
            ox, oy, oz = meta["origin"]

            pgm_path = os.path.join(os.path.dirname(yaml_path), img_name)
            gray = self._load_pgm(pgm_path)
            h, w = gray.shape

            # convert gray -> occupancy
            # black (0) -> occupied (100), white (255) -> free (0), others -> -1
            data = np.full((h, w), -1, dtype=np.int8)
            data[gray > 250] = 0
            data[gray < 50] = 100

            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.info.width = w
            msg.info.height = h
            msg.info.resolution = res
            msg.info.origin.position.x = ox
            msg.info.origin.position.y = oy
            msg.info.origin.position.z = oz
            msg.data = data.flatten().tolist()

            self.map_msg = msg
            self.map_frame = msg.header.frame_id

            # update grid + image
            grid = np.ones((h, w), dtype=np.uint8)
            grid[data == 0] = 0
            grid[data < 0] = 0
            grid[data > 50] = 1
            self.grid = grid

            img_color = np.zeros((h, w, 3), dtype=np.float32)
            img_color[:] = [0.5, 0.5, 0.5]
            img_color[data == 0] = [1.0, 1.0, 1.0]
            img_color[data > 50] = [0.0, 0.0, 0.0]
            self.map = np.flipud(img_color)

            self.get_logger().info(
                f"LOAD_MAP: Loaded {yaml_path} with image {pgm_path}"
            )
        except Exception as e:
            self.get_logger().error(f"LOAD_MAP: failed: {e}")

    def publish_path_and_goal(self, path_xy, frame_id: str):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for (x, y) in path_xy:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        goal_pose = path_msg.poses[-1]
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal_pose)

        self.get_logger().info(
            f"Published path with {len(path_msg.poses)} poses to /a_star_path"
        )

    # ===== vong lap ve anh =====

    def update_image(self):
        if self.map is None or self.map_msg is None:
            # van xu ly load/save neu map_msg None? save ko dc, load co the
            self.process_load_request_if_any()
            return

        # Thu tu: clear -> save -> load -> goal
        self.process_clear_request_if_any()
        self.process_save_request_if_any()
        self.process_load_request_if_any()
        self.process_goal_request_if_any()

        info = self.map_msg.info
        res = info.resolution
        w = info.width
        h = info.height
        ox = info.origin.position.x
        oy = info.origin.position.y

        extent = [ox, ox + w * res, oy, oy + h * res]

        fig, ax = plt.subplots(figsize=(4, 4), dpi=90)
        ax.imshow(self.map, origin="lower", extent=extent)
        ax.set_aspect("equal", adjustable="box")
        ax.set_axis_off()

        # ve path
        if self.path_xy and len(self.path_xy) >= 2:
            xs, ys = zip(*self.path_xy)
            ax.plot(xs, ys, "r-", linewidth=2)

        # ve robot + cap nhat CURRENT_POSE
        global CURRENT_POSE
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)

            CURRENT_POSE["x"] = float(x)
            CURRENT_POSE["y"] = float(y)
            CURRENT_POSE["theta"] = float(yaw)

            length = 0.3
            ax.add_patch(
                FancyArrow(
                    x,
                    y,
                    length * math.cos(yaw),
                    length * math.sin(yaw),
                    width=0.1 * res,
                    color="yellow",
                )
            )
        except Exception:
            pass

        fig.savefig(MAP_PNG_PATH, bbox_inches="tight", pad_inches=0)
        plt.close(fig)


def start_web_server():
    httpd = HTTPServer(("0.0.0.0", 8080), ImageServer)
    httpd.serve_forever()


def main():
    threading.Thread(target=start_web_server, daemon=True).start()
    rclpy.init()
    node = LiveMapWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
