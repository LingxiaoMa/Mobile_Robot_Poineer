import os
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from datetime import datetime
import message_filters

MIN_CONTOUR_AREA = 500
PHOTO_DIR = '/workspace/object_photos'
SAVE_COOLDOWN_S = 3.0

# HSV colour masks (OpenCV H: 0-179)
COLOURS = {
    'red':    [((0,   120, 80),  (10,  255, 255)),
               ((165, 120, 80),  (179, 255, 255))],
    'orange': [((11,  150, 100), (25,  255, 255))],
    'yellow': [((26,  100, 100), (34,  255, 255))],
    'green':  [((35,  80,  60),  (85,  255, 255))],
    'blue':   [((86,  80,  60),  (130, 255, 255))],
    'purple': [((131, 60,  50),  (164, 255, 255))],
    'white':  [((0,   0,   200), (179, 40,  255))],
}

CONTOUR_COLOURS = {
    'red':    (0,   0,   255),
    'orange': (0,   128, 255),
    'yellow': (0,   255, 255),
    'green':  (0,   255, 0),
    'blue':   (255, 128, 0),
    'purple': (255, 0,   255),
    'white':  (200, 200, 200),
}


def _build_mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lo), np.array(hi)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5),  np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    return mask


def _classify_shape(contour) -> str:
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return 'unknown'
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    area = cv2.contourArea(contour)
    circularity = 4 * np.pi * area / (peri * peri)
    n = len(approx)
    if circularity > 0.78:
        return 'circle'
    if n == 3:
        return 'triangle'
    if n == 4:
        x, y, w, h = cv2.boundingRect(approx)
        aspect = w / h if h > 0 else 1
        return 'square' if 0.85 < aspect < 1.15 else 'rectangle'
    if n == 5:
        return 'pentagon'
    if n == 6:
        return 'hexagon'
    return 'irregular'


def _sample_depth(depth, cx, cy, radius=5):
    h, w = depth.shape[:2]
    x0, x1 = max(cx - radius, 0), min(cx + radius, w)
    y0, y1 = max(cy - radius, 0), min(cy + radius, h)
    roi = depth[y0:y1, x0:x1].astype(np.float32)
    valid = roi[roi > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0   # mm → m


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self._bridge = CvBridge()
        self._last_save = 0.0
        if os.path.exists(PHOTO_DIR):
            import shutil
            shutil.rmtree(PHOTO_DIR)
        os.makedirs(PHOTO_DIR)

        self._info_pub = self.create_publisher(String, 'objects/info', 10)

        rgb_sub   = message_filters.Subscriber(self, Image, '/oak/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/oak/stereo/image_raw')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self._sync.registerCallback(self._callback)
        self.get_logger().info(f'ObjectDetector ready  photos→{PHOTO_DIR}')

    def _callback(self, rgb_msg: Image, depth_msg: Image):
        bgr   = self._bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        detections = []
        for colour, ranges in COLOURS.items():
            mask = _build_mask(hsv, ranges)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                dist_m = _sample_depth(depth, cx, cy)
                if dist_m is None:
                    continue
                shape = _classify_shape(c)
                detections.append((c, cx, cy, dist_m, colour, shape))

        if not detections:
            return

        parts = [f'{col} {shp} @ {d:.2f}m' for _, _, _, d, col, shp in detections]
        msg = String()
        msg.data = ' | '.join(parts)
        self._info_pub.publish(msg)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_save < SAVE_COOLDOWN_S:
            return
        self._last_save = now

        annotated = bgr.copy()
        for i, (contour, cx, cy, dist_m, colour, shape) in enumerate(detections):
            draw_col = CONTOUR_COLOURS.get(colour, (0, 255, 0))
            cv2.drawContours(annotated, [contour], -1, draw_col, 2)
            cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            label = f'{colour} {shape} {dist_m:.2f}m'
            cv2.putText(annotated, label, (cx - 60, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_col, 2)

        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(PHOTO_DIR, f'objects_{ts}.jpg')
        cv2.imwrite(path, annotated)
        self.get_logger().info(f'[{len(detections)} object(s)] {msg.data}  → {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
