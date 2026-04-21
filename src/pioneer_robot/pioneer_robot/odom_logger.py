import io
import threading

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from flask import Flask, Response

OUTPUT_FILE = '/workspace/odom_log.txt'


class OdomLogger(Node):

    def __init__(self):
        super().__init__('odom_logger')
        self._lock = threading.Lock()
        self._count = 0
        self._last_x = None
        self._last_y = None
        self._last_log_time = 0.0
        self._origin_x = None
        self._origin_y = None
        self._xs = []
        self._ys = []

        with open(OUTPUT_FILE, 'w') as f:
            f.write(f'{"#":<6} {"x (m)":>12} {"y (m)":>12}\n')
            f.write('-' * 32 + '\n')

        self.create_subscription(Odometry, '/odometry/filtered', self._callback, 10)
        self.get_logger().info(f'OdomLogger ready  →  {OUTPUT_FILE}  |  http://localhost:5001')

    def _callback(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time < 1.0:
            return

        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)

        if x == self._last_x and y == self._last_y:
            return

        self._last_x = x
        self._last_y = y
        self._last_log_time = now
        self._count += 1

        with open(OUTPUT_FILE, 'a') as f:
            f.write(f'{self._count:<6} {x:>12.4f} {y:>12.4f}\n')

        with self._lock:
            if self._origin_x is None:
                self._origin_x = x
                self._origin_y = y
            self._xs.append(x - self._origin_x)
            self._ys.append(y - self._origin_y)

    def get_path(self):
        with self._lock:
            return list(self._xs), list(self._ys)


def render_png(node: OdomLogger) -> bytes:
    xs, ys = node.get_path()

    fig, ax = plt.subplots(figsize=(7, 7), facecolor='#0f0f1a')
    ax.set_facecolor('#1a1a2e')
    ax.set_aspect('equal')
    ax.tick_params(colors='gray')
    ax.set_xlabel('X (m)', color='gray')
    ax.set_ylabel('Y (m)', color='gray')
    ax.set_title('Journey Summary', color='white', fontsize=14)
    for spine in ax.spines.values():
        spine.set_edgecolor('#444')

    if xs:
        margin = 0.5
        ax.set_xlim(min(0.0, min(xs)) - margin, max(0.0, max(xs)) + margin)
        ax.set_ylim(min(0.0, min(ys)) - margin, max(0.0, max(ys)) + margin)
        ax.plot(xs, ys, color='#00aaff', linewidth=1.5, zorder=2)
        ax.plot(xs[0],  ys[0],  'go', markersize=10, label='start',   zorder=3)
        ax.plot(xs[-1], ys[-1], 'r^', markersize=10, label='current', zorder=3)
        ax.legend(facecolor='#1a1a2e', labelcolor='white', fontsize=9)
    else:
        ax.set_xlim(-1, 5)
        ax.set_ylim(-1, 5)
        ax.text(2, 2, 'Waiting for odometry…', color='gray', ha='center', fontsize=12)

    ax.axhline(0, color='#444', linewidth=0.6)
    ax.axvline(0, color='#444', linewidth=0.6)

    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight', dpi=100)
    plt.close(fig)
    buf.seek(0)
    return buf.read()


HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Journey Summary</title>
  <style>
    body { background: #0f0f1a; display: flex; justify-content: center;
           align-items: center; height: 100vh; margin: 0; }
    img  { border: 1px solid #333; }
  </style>
</head>
<body>
  <img id="map" src="/path.png" width="700" height="700">
  <script>
    setInterval(() => {
      document.getElementById('map').src = '/path.png?t=' + Date.now();
    }, 500);
  </script>
</body>
</html>"""


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = Flask(__name__)

    @app.route('/')
    def index():
        return HTML_PAGE

    @app.route('/path.png')
    def path_png():
        return Response(render_png(node), mimetype='image/png')

    try:
        app.run(host='0.0.0.0', port=5001, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
