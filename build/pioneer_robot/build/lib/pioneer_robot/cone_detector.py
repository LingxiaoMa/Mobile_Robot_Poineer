import os
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from datetime import datetime
import message_filters

# Orange HSV range (OpenCV: H 0-179)
HSV_LOWER1 = np.array([0,  120, 100])   # red-orange (low)
HSV_UPPER1 = np.array([20, 255, 255])
HSV_LOWER2 = np.array([160, 120, 100])  # red wrap-around (high)
HSV_UPPER2 = np.array([179, 255, 255])

MIN_CONTOUR_AREA = 500   # px² — ignore small blobs
PHOTO_DIR = '/workspace/cone_photos'
SAVE_COOLDOWN_S = 5.0    # seconds between saves


class ConeDetector(Node):

    def __init__(self):
        super().__init__('cone_detector')
        self._bridge = CvBridge()
        self._last_save_time = 0.0
        if os.path.exists(PHOTO_DIR):
            import shutil
            shutil.rmtree(PHOTO_DIR)
        os.makedirs(PHOTO_DIR)

        self._dist_pub = self.create_publisher(Float32, 'cone/distance', 10)

        rgb_sub   = message_filters.Subscriber(self, Image, '/oak/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/oak/stereo/image_raw')

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self._sync.registerCallback(self._callback)
        self.get_logger().info(f'Cone detector ready. Photos → {PHOTO_DIR}')

    def _callback(self, rgb_msg: Image, depth_msg: Image):
        bgr   = self._bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # --- colour mask (orange + red wrap-around) ---
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, HSV_LOWER1, HSV_UPPER1)
        mask2 = cv2.inRange(hsv, HSV_LOWER2, HSV_UPPER2)
        mask  = cv2.bitwise_or(mask1, mask2)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]
        if not valid:
            return

        # --- process every cone ---
        detections = []
        for contour in valid:
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            dist_m = self._sample_depth(depth, cx, cy)
            if dist_m is None:
                continue
            shape = self._classify_shape(contour)
            detections.append((contour, cx, cy, dist_m, shape))

        if not detections:
            return

        # publish distance of nearest cone
        nearest = min(detections, key=lambda d: d[3])
        msg = Float32()
        msg.data = nearest[3]
        self._dist_pub.publish(msg)

        # --- throttled save ---
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_save_time < SAVE_COOLDOWN_S:
            return
        self._last_save_time = now

        annotated = bgr.copy()
        for i, (contour, cx, cy, dist_m, shape) in enumerate(detections):
            cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            label = f'#{i+1} {shape} {dist_m:.2f}m'
            cv2.putText(annotated, label, (cx - 60, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(PHOTO_DIR, f'cone_{ts}.jpg')
        cv2.imwrite(path, annotated)

        summary = '  |  '.join(f'#{i+1} {s} {d:.2f}m' for i, (_, _, _, d, s) in enumerate(detections))
        self.get_logger().info(f'Detected {len(detections)} cone(s): {summary}  photo={path}')

    def _sample_depth(self, depth, cx, cy, radius=5):
        h, w = depth.shape[:2]
        x0, x1 = max(cx - radius, 0), min(cx + radius, w)
        y0, y1 = max(cy - radius, 0), min(cy + radius, h)
        roi = depth[y0:y1, x0:x1].astype(np.float32)

        # OAK-D stereo depth is uint16 in mm; 0 = invalid
        valid = roi[roi > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0  # mm → m

    def _classify_shape(self, contour):
        peri    = cv2.arcLength(contour, True)
        approx  = cv2.approxPolyDP(contour, 0.04 * peri, True)
        n = len(approx)
        if n == 3:
            return 'triangle'
        elif n == 4:
            return 'rectangle'
        else:
            return 'cone'  # rounded / irregular → treat as cone


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
