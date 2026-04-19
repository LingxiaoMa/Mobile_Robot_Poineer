#!/usr/bin/env python3
"""
relative_waypoint_follower.py — Waypoint follower using relative coordinates

Reads waypoints from a CSV file as (x, y) offsets in metres relative to the
robot's startup position and heading. For each waypoint:
  1. Navigates to the waypoint via Nav2 NavigateToPose
  2. Triggers a photo when within photo_trigger_dist metres of the target
  3. Bypasses the target from the left side
After all waypoints, returns home and prints a mission summary.

CSV format  (see config/square_test.csv for example):
    # name, rel_x_m, rel_y_m, yaw_deg
    corner_1,  1.0,  0.0,   0.0
    corner_2,  1.0,  1.0,  90.0
    corner_3,  0.0,  1.0, 180.0
    corner_4,  0.0,  0.0, 270.0

  rel_x_m  — metres forward  from start (+) or backward (-)
  rel_y_m  — metres left     from start (+) or right    (-)
  yaw_deg  — final heading offset from start heading (CCW positive)
             use 999 to keep current heading

Parameters:
    waypoints_file       path to CSV        default: <pkg>/config/square_test.csv
    photo_trigger_dist   metres to target   default: 1.0
    bypass_lateral_dist  metres left        default: 1.5
    bypass_forward_dist  metres past target default: 0.5
    enable_bypass        do left bypass     default: True
    enable_photo         trigger photos     default: True

Usage:
    ros2 run pioneer_robot relative_waypoint_follower
    ros2 run pioneer_robot relative_waypoint_follower \\
        --ros-args -p waypoints_file:=/abs/path/to/file.csv
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RelativeWaypointFollower(Node):

    def __init__(self):
        super().__init__('relative_waypoint_follower')

        # ── parameters ────────────────────────────────────────────────────
        default_csv = os.path.join(
            get_package_share_directory('pioneer_robot'),
            'config', 'square_test.csv')
        self.declare_parameter('waypoints_file',      default_csv)
        self.declare_parameter('photo_trigger_dist',  1.0)
        self.declare_parameter('bypass_lateral_dist', 1.5)
        self.declare_parameter('bypass_forward_dist', 0.5)
        self.declare_parameter('enable_bypass',       True)
        self.declare_parameter('enable_photo',        True)

        self._csv_path      = self.get_parameter('waypoints_file').value
        self._photo_dist    = self.get_parameter('photo_trigger_dist').value
        self._lat_dist      = self.get_parameter('bypass_lateral_dist').value
        self._fwd_dist      = self.get_parameter('bypass_forward_dist').value
        self._do_bypass     = self.get_parameter('enable_bypass').value
        self._do_photo      = self.get_parameter('enable_photo').value

        # ── state ──────────────────────────────────────────────────────────
        self._lock     = threading.Lock()
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._odom_received = False

        # recorded at startup
        self._start_x   = 0.0
        self._start_y   = 0.0
        self._start_yaw = 0.0

        self._results: list[dict] = []

        # ── subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

        # ── publishers ─────────────────────────────────────────────────────
        self._photo_pub = self.create_publisher(String, '/take_photo', 10)

        # ── Nav2 ───────────────────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Relative Waypoint Follower ready.')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_x        = msg.pose.pose.position.x
            self._odom_y        = msg.pose.pose.position.y
            self._odom_yaw      = _quat_to_yaw(msg.pose.pose.orientation)
            self._odom_received = True

    def _pose(self):
        with self._lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    # ── relative → odom conversion ─────────────────────────────────────────

    def _rel_to_odom(self, rel_x, rel_y):
        """
        Convert (rel_x, rel_y) offsets in the robot's start frame to
        absolute odom-frame coordinates.
          rel_x > 0  → forward from start heading
          rel_y > 0  → left    from start heading
        """
        cos_y = math.cos(self._start_yaw)
        sin_y = math.sin(self._start_yaw)
        x = self._start_x + cos_y * rel_x - sin_y * rel_y
        y = self._start_y + sin_y * rel_x + cos_y * rel_y
        return x, y

    # ── helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _dist2d(x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _trigger_photo(self, label: str):
        if not self._do_photo:
            return
        msg = String()
        msg.data = label
        self._photo_pub.publish(msg)
        self.get_logger().info(f'  Photo triggered: {label}')

    # ── Nav2 ───────────────────────────────────────────────────────────────

    def _navigate_to(self, x, y, yaw, label='goal',
                     photo_dist: float | None = None) -> bool:
        if not self._nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        photo_done = [False]

        def _feedback_cb(_fb):
            if photo_dist and not photo_done[0]:
                rx, ry, _ = self._pose()
                if self._dist2d(rx, ry, x, y) <= photo_dist:
                    self._trigger_photo(label)
                    photo_done[0] = True

        self.get_logger().info(
            f'  -> {label}  odom({x:.2f}, {y:.2f})  yaw={math.degrees(yaw):.0f}°')
        future = self._nav.send_goal_async(goal, feedback_callback=_feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error(f'Goal rejected: {label}')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        ok = result_future.result().status == 4
        self.get_logger().info(
            f'  {"Reached" if ok else "FAILED"}: {label}')
        return ok

    def _bypass_left(self, wp_x, wp_y, label: str):
        if not self._do_bypass:
            return
        rx, ry, _ = self._pose()
        approach = math.atan2(wp_y - ry, wp_x - rx)
        left     = approach + math.pi / 2.0
        bx = wp_x + math.cos(left) * self._lat_dist + math.cos(approach) * self._fwd_dist
        by = wp_y + math.sin(left) * self._lat_dist + math.sin(approach) * self._fwd_dist
        self.get_logger().info(f'  Bypassing {label} left -> ({bx:.2f}, {by:.2f})')
        self._navigate_to(bx, by, approach, label=f'{label}_bypass')

    # ── CSV ────────────────────────────────────────────────────────────────

    def _load_csv(self) -> list[tuple[str, float, float, float]]:
        """Returns list of (name, rel_x, rel_y, yaw_deg)."""
        waypoints = []
        if not os.path.exists(self._csv_path):
            self.get_logger().error(f'CSV not found: {self._csv_path}')
            return waypoints
        with open(self._csv_path) as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                if len(row) < 3:
                    continue
                try:
                    name    = row[0].strip()
                    rel_x   = float(row[1])
                    rel_y   = float(row[2])
                    yaw_deg = float(row[3]) if len(row) >= 4 else 0.0
                    waypoints.append((name, rel_x, rel_y, yaw_deg))
                    self.get_logger().info(
                        f'  {name}: fwd={rel_x:.2f}m  left={rel_y:.2f}m  yaw={yaw_deg:.0f}°')
                except ValueError:
                    self.get_logger().warn(f'  Bad row: {row}')
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')
        return waypoints

    # ── startup ────────────────────────────────────────────────────────────

    def _wait_odom(self, timeout=10.0):
        self.get_logger().info('Waiting for odometry...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            with self._lock:
                if self._odom_received:
                    self.get_logger().info('Odometry ready.')
                    return True
        self.get_logger().warn('Odometry timeout — proceeding from (0,0).')
        return False

    # ── main ───────────────────────────────────────────────────────────────

    def run(self):
        # 1. Wait for Nav2 and odometry
        self.get_logger().info('Waiting for Nav2...')
        self._nav.wait_for_server()
        self.get_logger().info('Nav2 ready.')
        self._wait_odom()

        # 2. Record start pose
        sx, sy, syaw = self._pose()
        self._start_x   = sx
        self._start_y   = sy
        self._start_yaw = syaw
        self.get_logger().info(
            f'Start pose: odom({sx:.3f}, {sy:.3f})  yaw={math.degrees(syaw):.1f}°')

        # 3. Load CSV
        waypoints = self._load_csv()
        if not waypoints:
            return

        # 4. Convert relative → odom
        goals: list[tuple[str, float, float, float]] = []
        for name, rx, ry, yaw_deg in waypoints:
            abs_x, abs_y = self._rel_to_odom(rx, ry)
            if yaw_deg == 999.0:
                abs_yaw = syaw
            else:
                abs_yaw = syaw + math.radians(yaw_deg)
            goals.append((name, abs_x, abs_y, abs_yaw))

        self.get_logger().info('\nWaypoints in odom frame:')
        for name, gx, gy, gyaw in goals:
            self.get_logger().info(
                f'  {name}: ({gx:.2f}, {gy:.2f})  yaw={math.degrees(gyaw):.0f}°')

        # 5. Navigate each waypoint
        for (name, rx, ry, _), (_, gx, gy, gyaw) in zip(waypoints, goals):
            self.get_logger().info(f'\n{"─"*40}\nWaypoint: {name}\n{"─"*40}')
            reached = self._navigate_to(
                gx, gy, gyaw, label=name,
                photo_dist=self._photo_dist if self._do_photo else None)
            self._results.append({
                'name': name, 'rel_x': rx, 'rel_y': ry,
                'abs_x': gx, 'abs_y': gy, 'reached': reached})
            self._bypass_left(gx, gy, name)

        # 6. Return home
        self.get_logger().info(f'\n{"─"*40}\nReturning home\n{"─"*40}')
        self._navigate_to(sx, sy, syaw, label='home')

        # 7. Summary
        self._print_summary()

    # ── summary ────────────────────────────────────────────────────────────

    def _print_summary(self):
        ok = sum(1 for r in self._results if r['reached'])
        lines = [
            '',
            '=' * 50,
            'MISSION SUMMARY',
            '=' * 50,
            f'  Waypoints : {ok}/{len(self._results)} reached',
            f'  Photo dist: {self._photo_dist:.1f} m',
            f'  Bypass    : {"ON" if self._do_bypass else "OFF"}  '
            f'({self._lat_dist:.1f}m left, {self._fwd_dist:.1f}m fwd)',
            '─' * 50,
        ]
        for r in self._results:
            tick = 'OK  ' if r['reached'] else 'MISS'
            lines.append(
                f'  [{tick}] {r["name"]:20s}'
                f'  rel({r["rel_x"]:+.1f}, {r["rel_y"]:+.1f})'
                f'  odom({r["abs_x"]:.2f}, {r["abs_y"]:.2f})')
        lines += ['─' * 50, '  Returned to home.', '=' * 50]
        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = RelativeWaypointFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
