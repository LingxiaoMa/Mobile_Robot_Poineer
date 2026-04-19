#!/usr/bin/env python3
"""
gps_waypoint_follower.py — GPS waypoint follower for Pioneer 3-AT (real hardware)

Reads GPS waypoints from a CSV file, converts them to odom-frame coordinates
relative to the robot's startup position, then for each waypoint:
  1. Navigates to the waypoint via Nav2 NavigateToPose
  2. Triggers a photo when within 1 m of the target
  3. Bypasses the target from the left side
After all waypoints, returns home and prints a mission summary.

CSV format  (save to config/waypoints.csv):
    # name, latitude, longitude
    cone_1, -31.9804730, 115.8193318
    cone_2, -31.9805359, 115.8193635
    cone_3, -31.9804461, 115.8193530

Parameters:
    waypoints_file       path to CSV           default: <pkg>/config/waypoints.csv
    photo_trigger_dist   metres to target      default: 1.0
    bypass_lateral_dist  metres left offset    default: 1.5
    bypass_forward_dist  metres past target    default: 0.5

Usage:
    ros2 run pioneer_robot gps_waypoint_follower
    ros2 run pioneer_robot gps_waypoint_follower \\
        --ros-args -p waypoints_file:=/abs/path/to/waypoints.csv
"""

import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

_METERS_PER_DEG_LAT = 111320.0


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # ── parameters ────────────────────────────────────────────────────
        default_csv = os.path.join(
            get_package_share_directory('pioneer_robot'), 'config', 'waypoints.csv')
        self.declare_parameter('waypoints_file',      default_csv)
        self.declare_parameter('photo_trigger_dist',  1.0)
        self.declare_parameter('bypass_lateral_dist', 1.5)
        self.declare_parameter('bypass_forward_dist', 0.5)

        self._csv_path   = self.get_parameter('waypoints_file').value
        self._photo_dist = self.get_parameter('photo_trigger_dist').value
        self._lat_dist   = self.get_parameter('bypass_lateral_dist').value
        self._fwd_dist   = self.get_parameter('bypass_forward_dist').value

        # ── state ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._gps: NavSatFix | None = None
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        # recorded at startup
        self._origin_lat = 0.0
        self._origin_lon = 0.0
        self._origin_x   = 0.0
        self._origin_y   = 0.0
        self._origin_yaw = 0.0

        self._results: list[dict] = []

        # ── subscribers ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, '/fix',               self._gps_cb,  10)
        self.create_subscription(Odometry,  '/odometry/filtered', self._odom_cb, 10)

        # ── publishers ─────────────────────────────────────────────────────
        self._photo_pub = self.create_publisher(String, '/take_photo', 10)

        # ── Nav2 action client ─────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('GPS Waypoint Follower ready.')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self._gps = msg

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_x   = msg.pose.pose.position.x
            self._odom_y   = msg.pose.pose.position.y
            self._odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)

    def _pose(self):
        with self._lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    # ── GPS → odom conversion ──────────────────────────────────────────────

    def _ll_to_odom(self, lat, lon):
        """
        Convert GPS lat/lon to odom-frame x/y.
        Uses the startup GPS fix as origin and the startup odom heading to
        rotate ENU deltas into the odom frame.
        """
        mpd_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(self._origin_lat))
        d_east  = (lon - self._origin_lon) * mpd_lon
        d_north = (lat - self._origin_lat) * _METERS_PER_DEG_LAT

        # Rotate ENU (East, North) → odom (forward, left) using startup yaw.
        # ROS yaw is CCW from East, so East aligns with yaw=0.
        cos_y = math.cos(self._origin_yaw)
        sin_y = math.sin(self._origin_yaw)
        x = self._origin_x + cos_y * d_east + sin_y * d_north
        y = self._origin_y - sin_y * d_east + cos_y * d_north
        return x, y

    # ── helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _dist2d(x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def _make_pose(self, x, y, yaw=0.0):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _trigger_photo(self, label: str):
        msg = String()
        msg.data = label
        self._photo_pub.publish(msg)
        self.get_logger().info(f'  Photo triggered at {label}')

    # ── Nav2 ───────────────────────────────────────────────────────────────

    def _navigate_to(self, x, y, yaw=0.0, label='goal',
                     photo_at_dist: float | None = None,
                     photo_label: str = '') -> bool:
        """
        Send a NavigateToPose goal to (x, y, yaw).
        If photo_at_dist is set, publishes to /take_photo once when the
        robot enters that radius around the target.
        Returns True on SUCCEEDED.
        """
        if not self._nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        photo_done = [False]

        def _feedback_cb(_fb):
            if photo_at_dist and not photo_done[0]:
                rx, ry, _ = self._pose()
                if self._dist2d(rx, ry, x, y) <= photo_at_dist:
                    self._trigger_photo(photo_label or label)
                    photo_done[0] = True

        self.get_logger().info(f'Navigating → {label}  odom({x:.2f}, {y:.2f})')
        future = self._nav.send_goal_async(goal, feedback_callback=_feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error(f'Goal rejected: {label}')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        succeeded = result_future.result().status == 4
        if succeeded:
            self.get_logger().info(f'  Reached {label}')
        else:
            status = result_future.result().status
            self.get_logger().warn(f'  Did not reach {label} (status={status})')
        return succeeded

    def _bypass_left(self, wp_x, wp_y, label: str):
        """Navigate to a point that's bypass_lateral_dist m to the left of
        the waypoint and bypass_forward_dist m past it."""
        rx, ry, _ = self._pose()
        approach = math.atan2(wp_y - ry, wp_x - rx)
        left     = approach + math.pi / 2.0

        bx = wp_x + math.cos(left) * self._lat_dist + math.cos(approach) * self._fwd_dist
        by = wp_y + math.sin(left) * self._lat_dist + math.sin(approach) * self._fwd_dist

        self.get_logger().info(
            f'  Bypassing {label} left  odom({bx:.2f}, {by:.2f})')
        self._navigate_to(bx, by, yaw=approach, label=f'{label}_bypass')

    # ── CSV ────────────────────────────────────────────────────────────────

    def _load_csv(self) -> list[tuple[str, float, float]]:
        waypoints = []
        if not os.path.exists(self._csv_path):
            self.get_logger().error(f'Waypoints CSV not found: {self._csv_path}')
            return waypoints
        with open(self._csv_path) as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                if len(row) < 3:
                    continue
                try:
                    name = row[0].strip()
                    lat  = float(row[1])
                    lon  = float(row[2])
                    waypoints.append((name, lat, lon))
                    self.get_logger().info(
                        f'  Loaded: {name}  ({lat:.7f}, {lon:.7f})')
                except ValueError:
                    self.get_logger().warn(f'  Skipping bad row: {row}')
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')
        return waypoints

    # ── startup waits ──────────────────────────────────────────────────────

    def _wait_gps(self, timeout=60.0) -> NavSatFix | None:
        self.get_logger().info('Waiting for GPS fix...')
        deadline = time.time() + timeout
        last_log = 0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            with self._lock:
                fix = self._gps
            if fix and fix.status.status >= 0:
                self.get_logger().info(
                    f'GPS fix: lat={fix.latitude:.7f}  lon={fix.longitude:.7f}')
                return fix
            elapsed = int(time.time() - (deadline - timeout))
            if elapsed - last_log >= 5:
                last_log = elapsed
                self.get_logger().info(f'  waiting for GPS... ({elapsed}s)')
        self.get_logger().error('GPS fix timeout — aborting.')
        return None

    def _wait_odom(self, timeout=10.0):
        self.get_logger().info('Waiting for odometry...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('Odometry ready.')

    # ── main ───────────────────────────────────────────────────────────────

    def run(self):
        # 1. Wait for Nav2
        self.get_logger().info('Waiting for Nav2...')
        self._nav.wait_for_server()
        self.get_logger().info('Nav2 ready.')

        # 2. Wait for GPS fix
        fix = self._wait_gps()
        if fix is None:
            return

        # 3. Wait for odometry to stabilise, then record origin
        self._wait_odom()
        ox, oy, oyaw = self._pose()
        self._origin_lat = fix.latitude
        self._origin_lon = fix.longitude
        self._origin_x   = ox
        self._origin_y   = oy
        self._origin_yaw = oyaw
        self.get_logger().info(
            f'Origin: GPS({fix.latitude:.7f}, {fix.longitude:.7f})  '
            f'odom({ox:.3f}, {oy:.3f})  yaw={math.degrees(oyaw):.1f}°')

        # 4. Load CSV
        waypoints = self._load_csv()
        if not waypoints:
            return

        # 5. Convert GPS → odom
        goals: list[tuple[str, float, float]] = []
        for name, lat, lon in waypoints:
            x, y = self._ll_to_odom(lat, lon)
            goals.append((name, x, y))
            self.get_logger().info(f'  {name}: odom({x:.2f}, {y:.2f})')

        # 6. Navigate each waypoint
        for name, gx, gy in goals:
            self.get_logger().info(f'\n{"─"*45}\nWaypoint: {name}\n{"─"*45}')
            reached = self._navigate_to(
                gx, gy, label=name,
                photo_at_dist=self._photo_dist,
                photo_label=name)
            self._results.append({'name': name, 'x': gx, 'y': gy, 'reached': reached})
            self._bypass_left(gx, gy, name)

        # 7. Return home
        self.get_logger().info(f'\n{"─"*45}\nReturning home\n{"─"*45}')
        self._navigate_to(ox, oy, yaw=oyaw, label='home')

        # 8. Summary
        self._print_summary(waypoints, goals)

    # ── summary ────────────────────────────────────────────────────────────

    def _print_summary(self, waypoints, goals):
        reached_count = sum(1 for r in self._results if r['reached'])
        lines = [
            '',
            '=' * 55,
            'MISSION SUMMARY',
            '=' * 55,
            f'  Waypoints attempted : {len(self._results)}',
            f'  Waypoints reached   : {reached_count}',
            f'  Photo trigger radius: {self._photo_dist:.1f} m',
            f'  Left bypass offset  : {self._lat_dist:.1f} m lateral, '
            f'{self._fwd_dist:.1f} m forward',
            '─' * 55,
        ]
        for (name, lat, lon), (_, gx, gy) in zip(waypoints, goals):
            r = next((r for r in self._results if r['name'] == name), None)
            tick = 'OK' if (r and r['reached']) else 'MISS'
            lines.append(
                f'  [{tick:4s}] {name:20s}'
                f'  GPS({lat:.6f}, {lon:.6f})'
                f'  odom({gx:.1f}, {gy:.1f})')
        lines += [
            '─' * 55,
            '  Returned to home position.',
            '=' * 55,
        ]
        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
