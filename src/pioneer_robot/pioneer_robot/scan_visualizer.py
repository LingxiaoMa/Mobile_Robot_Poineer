#!/usr/bin/env python3
"""
实时激光雷达可视化节点（Web版，适用于无显示器环境）

订阅 /scan (sensor_msgs/LaserScan)，通过 Flask 网页实时显示扫描点。
浏览器访问：http://<机器人IP>:5000

运行方式：
    ros2 run pioneer_robot scan_visualizer
"""

import io
import math
import threading

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from flask import Flask, Response

# ---------- ROS2 节点 ----------

class ScanVisualizer(Node):
    def __init__(self):
        super().__init__('scan_visualizer')
        self._lock = threading.Lock()
        self._ranges = []
        self._angle_min = 0.0
        self._angle_inc = 0.0
        self._range_max = 10.0
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.get_logger().info('ScanVisualizer started — open http://localhost:5000 in browser')

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._ranges     = list(msg.ranges)
            self._angle_min  = msg.angle_min
            self._angle_inc  = msg.angle_increment
            self._range_max  = msg.range_max if msg.range_max < 50.0 else 10.0

    def get_scan_data(self):
        with self._lock:
            return (
                list(self._ranges),
                self._angle_min,
                self._angle_inc,
                self._range_max,
            )


# ---------- 渲染图像 ----------

def render_png(node: ScanVisualizer) -> bytes:
    ranges, angle_min, angle_inc, range_max = node.get_scan_data()

    xs, ys = [], []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and 0.01 < r < range_max:
            a = angle_min + i * angle_inc
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))

    lim = min(range_max, 6.0)
    fig, ax = plt.subplots(figsize=(7, 7), facecolor='#0f0f1a')
    ax.set_facecolor('#1a1a2e')
    ax.set_aspect('equal')
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_title('激光雷达实时扫描', color='white', fontsize=14)
    ax.tick_params(colors='gray')
    ax.set_xlabel('X (m)', color='gray')
    ax.set_ylabel('Y (m)', color='gray')
    for spine in ax.spines.values():
        spine.set_edgecolor('#444')

    for r in [1, 2, 3, 4, 5]:
        if r <= lim:
            ax.add_patch(plt.Circle((0, 0), r, fill=False, color='#333355', linewidth=0.8))
            ax.text(0, r + 0.05, f'{r}m', color='#555577', fontsize=7, ha='center')

    if xs:
        ax.scatter(xs, ys, s=4, c='#00ff88', alpha=0.85, linewidths=0)

    ax.plot(0, 0, 'w^', markersize=10, zorder=5)

    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight', dpi=100)
    plt.close(fig)
    buf.seek(0)
    return buf.read()


# ---------- Flask 服务 ----------

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>激光雷达可视化</title>
  <style>
    body { background: #0f0f1a; display: flex; justify-content: center;
           align-items: center; height: 100vh; margin: 0; }
    img  { border: 1px solid #333; }
  </style>
</head>
<body>
  <img id="scan" src="/scan.png" width="700" height="700">
  <script>
    setInterval(() => {
      document.getElementById('scan').src = '/scan.png?t=' + Date.now();
    }, 200);
  </script>
</body>
</html>"""


def create_app(node: ScanVisualizer) -> Flask:
    app = Flask(__name__)

    @app.route('/')
    def index():
        return HTML_PAGE

    @app.route('/scan.png')
    def scan_png():
        png = render_png(node)
        return Response(png, mimetype='image/png')

    return app


# ---------- 入口 ----------

def main():
    rclpy.init()
    node = ScanVisualizer()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = create_app(node)
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
