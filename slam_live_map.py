#!/usr/bin/env python3
import os
import math
import threading
import json
from http.server import SimpleHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

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

GOAL_REQUEST = None
CLEAR_REQUEST = False
LOCK = threading.Lock()

# Pose hiá»‡n táº¡i cá»§a robot trong frame "map"
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


class ImageServer(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # ============== HTML chĂ­nh ==============
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
      align-items: center; /* map náº±m giá»¯a khung iframe */
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
    }
    .btn {
      padding: 8px 14px;
      font-size: 14px;
      background: #c00;
      color: #fff;
      border: none;
      border-radius: 4px;
      cursor: pointer;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="toolbar">
      <button class="btn"
        onclick="fetch('/clear_path').catch(()=>{});">
        STOP & CLEAR PATH
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

    // click chá»n goal
    img.addEventListener("click", (e) => {
      const rect = img.getBoundingClientRect();
      const ux = (e.clientX - rect.left) / rect.width;
      const uy_screen = (e.clientY - rect.top) / rect.height;
      const v = 1.0 - uy_screen;
      fetch(`/set_goal?u=${ux}&v=${v}`).catch(() => {});
    });
  </script>
</body>
</html>
"""
            self.wfile.write(html.encode("utf-8"))
            return

        # ============== áº¢nh map ==============
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

        # ============== API POSE ==============
        if path == "/pose":
            # tráº£ vá» JSON: {"x": ..., "y": ..., "theta": ...}
            body = json.dumps(CURRENT_POSE).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # ============== Set goal tá»« web ==============
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

        # ============== NĂºt STOP & CLEAR PATH ==============
        if path.startswith("/clear_path"):
            request_clear_path()
            self.send_response(200)
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write("CLEARED".encode("utf-8"))
            return

        # ============== Máº·c Ä‘á»‹nh 404 ==============
        self.send_error(404)


class LiveMapWeb(Node):
    def __init__(self):
        super().__init__("slam_live_map_viewer")

        self.map_topic  = os.environ.get("MAP_TOPIC", "/map")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")

        self.map       = None
        self.map_msg   = None
        self.map_frame = "map"
        self.grid      = None
        self.path_xy   = None

        os.makedirs(BASE_DIR, exist_ok=True)

        self.tf_buffer   = Buffer()
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

        # Publish path + goal cho follower
        self.path_pub = self.create_publisher(Path, "/a_star_path", 1)
        self.goal_pub = self.create_publisher(PoseStamped, "/a_star_goal", 1)

        self.get_logger().info(
            f"[slam_live_map] HTTP: http://<IP_RPI>:8080/ | subscribe {self.map_topic}"
        )

        self.timer = self.create_timer(0.5, self.update_image)

    def cb_map(self, msg: OccupancyGrid):
        self.map_msg   = msg
        self.map_frame = msg.header.frame_id or "map"

        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # Grid 0=free, 1=obstacle/unknown
        grid = np.ones((h, w), dtype=np.uint8)
        grid[data == 0] = 0
        grid[data < 0]  = 0
        grid[data > 50] = 1
        self.grid = grid

        img = np.zeros((h, w, 3), dtype=np.float32)
        img[:] = [0.5, 0.5, 0.5]
        img[data == 0] = [1.0, 1.0, 1.0]
        img[data > 50] = [0.0, 0.0, 0.0]
        self.map = np.flipud(img)

    # ========= xá»­ lĂ½ cĂ¡c request tá»« web =========

    def process_goal_request_if_any(self):
        req = pop_goal_request()
        if req is None or self.map_msg is None or self.grid is None or self.map is None:
            return

        u, v = req
        info = self.map_msg.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

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
                f"KhĂ´ng láº¥y Ä‘Æ°á»£c TF map->{self.base_frame}: {e}"
            )
            return

        self.get_logger().info(
            f"Click u={u:.3f}, v={v:.3f} -> cell=({r_grid},{c_grid}), "
            f"goal_world=({gx:.2f},{gy:.2f})"
        )

        # TĂ­nh path
        self.path_xy = plan_path(
            self.grid,
            info,
            (rx, ry),
            (gx, gy),
            logger=self.get_logger(),
        )

        # Publish path Ä‘á»ƒ robot bĂ¡m
        if self.path_xy and len(self.path_xy) >= 2:
            self.publish_path_and_goal(self.path_xy, self.map_frame)

    def process_clear_request_if_any(self):
        if not pop_clear_request():
            return

        # XĂ³a path trong viewer
        self.path_xy = None

        # Publish path rá»—ng Ä‘á»ƒ follower dá»«ng
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        self.path_pub.publish(path_msg)

        self.get_logger().info("Clear path request: published empty /a_star_path.")

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

    # ========= vĂ²ng láº·p váº½ áº£nh =========

    def update_image(self):
        if self.map is None or self.map_msg is None:
            return

        # Xá»­ lĂ½ STOP trÆ°á»›c, rá»“i goal
        self.process_clear_request_if_any()
        self.process_goal_request_if_any()

        info = self.map_msg.info
        res  = info.resolution
        w    = info.width
        h    = info.height
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        extent = [ox, ox + w * res, oy, oy + h * res]

        fig, ax = plt.subplots(figsize=(4, 4), dpi=90)
        ax.imshow(self.map, origin="lower", extent=extent)
        ax.set_aspect("equal", adjustable="box")
        ax.set_axis_off()

        # Váº½ path
        if self.path_xy and len(self.path_xy) >= 2:
            xs, ys = zip(*self.path_xy)
            ax.plot(xs, ys, "r-", linewidth=2)

        # Váº½ robot + cáº­p nháº­t CURRENT_POSE
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

            # lÆ°u pose cho HTTP /pose
            CURRENT_POSE["x"] = float(x)
            CURRENT_POSE["y"] = float(y)
            CURRENT_POSE["theta"] = float(yaw)

            length = 0.3
            ax.add_patch(FancyArrow(
                x, y,
                length * math.cos(yaw),
                length * math.sin(yaw),
                width=0.1 * res,
                color="yellow",
            ))
        except Exception:
            # náº¿u khĂ´ng láº¥y Ä‘Æ°á»£c TF thĂ¬ giá»¯ nguyĂªn pose cÅ©
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

