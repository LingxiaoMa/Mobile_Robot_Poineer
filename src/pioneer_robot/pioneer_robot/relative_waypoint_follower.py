#!/usr/bin/env python3
"""
relative_waypoint_follower.py — Waypoint follower using relative coordinates

Waits for a "START" message on /mission_control (published by joy_controller
when the X button is pressed) before running the mission. Pressing O sends
"CANCEL" which aborts the current Nav2 goal and stops the mission.

CSV format  (see config/square_test.csv for example):
    # name, rel_x_m, rel_y_m, yaw_deg
    corner_1,  1.0,  0.0,   0.0

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
    # Then press X on joystick to start, O to cancel.
"""

import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

        self._cb_group = ReentrantCallbackGroup()

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

        self._csv_path   = self.get_parameter('waypoints_file').value
        self._photo_dist = self.get_parameter('photo_trigger_dist').value
        self._lat_dist   = self.get_parameter('bypass_lateral_dist').value
        self._fwd_dist   = self.get_parameter('bypass_forward_dist').value
        self._do_bypass  = self.get_parameter('enable_bypass').value
        self._do_photo   = self.get_parameter('enable_photo').value

        # ── state ──────────────────────────────────────────────────────────
        self._lock              = threading.Lock()
        self._odom_x            = 0.0
        self._odom_y            = 0.0
        self._odom_yaw          = 0.0
        self._odom_received     = False
        self._start_x           = 0.0
        self._start_y           = 0.0
        self._start_yaw         = 0.0
        self._results: list[dict] = []

        # mission control
        self._start_event       = threading.Event()
        self._cancel_flag       = False
        self._active_handle     = None   # current Nav2 goal handle
        self._mission_running   = False

        # ── subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/mission_control', self._mission_cb, 10,
            callback_group=self._cb_group)

        # ── publishers ─────────────────────────────────────────────────────
        self._photo_pub = self.create_publisher(String, '/take_photo', 10)

        # ── Nav2 ───────────────────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                 callback_group=self._cb_group)

        self.get_logger().info(
            'Relative Waypoint Follower ready.\n'
            '  Press X on joystick to START mission.\n'
            '  Press O on joystick to CANCEL and switch to manual.')

    # ── mission control callback ────────────────────────────────────────────

    def _mission_cb(self, msg: String):
        if msg.data == 'START':
            if self._mission_running:
                self.get_logger().warn('Mission already running — ignoring START')
                return
            self.get_logger().info('START received — beginning mission.')
            self._cancel_flag = False
            self._results.clear()
            self._start_event.set()

        elif msg.data == 'CANCEL':
            if not self._mission_running:
                return
            self.get_logger().info('CANCEL received — aborting mission.')
            self._cancel_flag = True
            with self._lock:
                handle = self._active_handle
            if handle is not None:
                handle.cancel_goal_async()

    # ── odometry ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_x        = msg.pose.pose.position.x
            self._odom_y        = msg.pose.pose.position.y
            self._odom_yaw      = _quat_to_yaw(msg.pose.pose.orientation)
            self._odom_received = True

    def _pose(self):
        with self._lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    # ── coordinate conversion ───────────────────────────────────────────────

    def _rel_to_odom(self, rel_x, rel_y):
        cos_y = math.cos(self._start_yaw)
        sin_y = math.sin(self._start_yaw)
        x = self._start_x + cos_y * rel_x - sin_y * rel_y
        y = self._start_y + sin_y * rel_x + cos_y * rel_y
        return x, y

    # ── helpers ─────────────────────────────────────────────────────────────

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

    # ── Nav2 navigation ─────────────────────────────────────────────────────

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

        # Busy-wait so the MultiThreadedExecutor can process callbacks freely
        while not future.done():
            time.sleep(0.05)
            if self._cancel_flag:
                return False

        handle = future.result()
        with self._lock:
            self._active_handle = handle

        if not handle.accepted:
            self.get_logger().error(f'Goal rejected: {label}')
            with self._lock:
                self._active_handle = None
            return False

        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)
            if self._cancel_flag:
                with self._lock:
                    self._active_handle = None
                return False

        with self._lock:
            self._active_handle = None

        ok = result_future.result().status == 4
        self.get_logger().info(f'  {"Reached" if ok else "FAILED"}: {label}')
        return ok

    def _bypass_left(self, wp_x, wp_y, label: str):
        if not self._do_bypass or self._cancel_flag:
            return
        rx, ry, _ = self._pose()
        approach = math.atan2(wp_y - ry, wp_x - rx)
        left     = approach + math.pi / 2.0
        bx = wp_x + math.cos(left) * self._lat_dist + math.cos(approach) * self._fwd_dist
        by = wp_y + math.sin(left) * self._lat_dist + math.sin(approach) * self._fwd_dist
        self.get_logger().info(f'  Bypassing {label} left -> ({bx:.2f}, {by:.2f})')
        self._navigate_to(bx, by, approach, label=f'{label}_bypass')

    # ── CSV loading ─────────────────────────────────────────────────────────

    def _load_csv(self) -> list[tuple[str, float, float, float]]:
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

    # ── wait helpers ─────────────────────────────────────────────────────────

    def _wait_odom(self, timeout=10.0):
        self.get_logger().info('Waiting for odometry...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.2)
            with self._lock:
                if self._odom_received:
                    self.get_logger().info('Odometry ready.')
                    return True
        self.get_logger().warn('Odometry timeout — proceeding from (0,0).')
        return False

    # ── mission ──────────────────────────────────────────────────────────────

    def _run_mission(self):
        self._mission_running = True
        try:
            self.get_logger().info('Waiting for Nav2...')
            self._nav.wait_for_server()
            self.get_logger().info('Nav2 ready.')
            self._wait_odom()

            sx, sy, syaw = self._pose()
            self._start_x   = sx
            self._start_y   = sy
            self._start_yaw = syaw
            self.get_logger().info(
                f'Start pose: odom({sx:.3f}, {sy:.3f})  yaw={math.degrees(syaw):.1f}°')

            waypoints = self._load_csv()
            if not waypoints:
                return

            goals: list[tuple[str, float, float, float]] = []
            for name, rx, ry, yaw_deg in waypoints:
                abs_x, abs_y = self._rel_to_odom(rx, ry)
                abs_yaw = syaw if yaw_deg == 999.0 else syaw + math.radians(yaw_deg)
                goals.append((name, abs_x, abs_y, abs_yaw))

            for (name, rx, ry, _), (_, gx, gy, gyaw) in zip(waypoints, goals):
                if self._cancel_flag:
                    self.get_logger().info('Mission cancelled.')
                    break
                self.get_logger().info(f'\n{"─"*40}\nWaypoint: {name}\n{"─"*40}')
                reached = self._navigate_to(
                    gx, gy, gyaw, label=name,
                    photo_dist=self._photo_dist if self._do_photo else None)
                self._results.append({
                    'name': name, 'rel_x': rx, 'rel_y': ry,
                    'abs_x': gx, 'abs_y': gy, 'reached': reached})
                self._bypass_left(gx, gy, name)

            if not self._cancel_flag:
                self.get_logger().info(f'\n{"─"*40}\nReturning home\n{"─"*40}')
                self._navigate_to(sx, sy, syaw, label='home')

            self._print_summary()
        finally:
            self._mission_running = False
            self._start_event.clear()

    def _print_summary(self):
        ok = sum(1 for r in self._results if r['reached'])
        status = 'CANCELLED' if self._cancel_flag else 'COMPLETE'
        lines = [
            '', '=' * 50,
            f'MISSION {status}',
            '=' * 50,
            f'  Waypoints : {ok}/{len(self._results)} reached',
            f'  Bypass    : {"ON" if self._do_bypass else "OFF"}',
            '─' * 50,
        ]
        for r in self._results:
            tick = 'OK  ' if r['reached'] else 'MISS'
            lines.append(
                f'  [{tick}] {r["name"]:20s}'
                f'  rel({r["rel_x"]:+.1f}, {r["rel_y"]:+.1f})'
                f'  odom({r["abs_x"]:.2f}, {r["abs_y"]:.2f})')
        lines += ['─' * 50, '=' * 50]
        for line in lines:
            self.get_logger().info(line)

    # ── wait loop (called from main thread) ──────────────────────────────────

    def wait_and_run(self):
        while rclpy.ok():
            self.get_logger().info('Waiting for X button (START)...')
            self._start_event.wait()
            if not rclpy.ok():
                break
            self._run_mission()


def main(args=None):
    rclpy.init(args=args)
    node = RelativeWaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # spin executor in background thread; wait_and_run blocks main thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.wait_and_run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
