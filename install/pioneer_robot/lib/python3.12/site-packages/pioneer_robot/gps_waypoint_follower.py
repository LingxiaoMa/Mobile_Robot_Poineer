#!/usr/bin/env python3
"""
gps_waypoint_follower.py — GPS waypoint follower for Pioneer 3-AT (real hardware)

Waits for "START" on /mission_control (joy_controller X + L1+L2) before running.
Releasing L1+L2 or pressing O cancels the mission immediately.

Progress is saved to /workspace/mission_progress.json after each waypoint.
On restart the mission resumes from the first unvisited waypoint.

CSV location (edit source, then rebuild):
    src/pioneer_robot/config/waypoints.csv

CSV format:
    # name, latitude, longitude
    cone_1, -31.9804730, 115.8193318

  Coordinates are absolute GPS (decimal degrees).
  At startup the robot records its GPS fix as origin; all waypoints are
  converted to odom frame relative to that origin using flat-earth ENU.

Photo behaviour:
  After reaching each waypoint the robot turns 90° right in place,
  publishes to /take_photo, then continues to the next waypoint.

Parameters:
    waypoints_file       path to CSV           default: <pkg>/config/waypoints.csv
    bypass_lateral_dist  metres left offset    default: 1.5
    bypass_forward_dist  metres past target    default: 0.5
    enable_bypass        do left bypass        default: True
    enable_photo         turn+photo at target  default: True

Usage:
    ros2 run pioneer_robot gps_waypoint_follower
    # Hold L1+L2, press X to start. Press O to cancel.
"""

import csv
import json
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
from sensor_msgs.msg import NavSatFix, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

BUTTON_L1              = 9
BUTTON_L2              = 10
PROGRESS_FILE          = '/workspace/mission_progress.json'
GOAL_FRAME             = 'odom'
_METERS_PER_DEG_LAT    = 111320.0


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        self._cb_group = ReentrantCallbackGroup()

        # ── parameters ────────────────────────────────────────────────────
        default_csv = os.path.join(
            get_package_share_directory('pioneer_robot'), 'config', 'waypoints.csv')
        self.declare_parameter('waypoints_file',      default_csv)
        self.declare_parameter('bypass_lateral_dist', 1.5)
        self.declare_parameter('bypass_forward_dist', 0.5)
        self.declare_parameter('enable_bypass',       True)
        self.declare_parameter('enable_photo',        True)
        self.declare_parameter('initial_heading_deg', 90.0)  # 90=北 0=东 -90=南 180=西

        self._csv_path      = self.get_parameter('waypoints_file').value
        self._lat_dist      = self.get_parameter('bypass_lateral_dist').value
        self._fwd_dist      = self.get_parameter('bypass_forward_dist').value
        self._do_bypass     = self.get_parameter('enable_bypass').value
        self._do_photo      = self.get_parameter('enable_photo').value
        self._initial_heading = math.radians(
            self.get_parameter('initial_heading_deg').value)

        # ── state ──────────────────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._gps           = None
        self._odom_x        = 0.0
        self._odom_y        = 0.0
        self._odom_yaw      = 0.0
        self._origin_lat    = 0.0
        self._origin_lon    = 0.0
        self._origin_x      = 0.0
        self._origin_y      = 0.0
        self._origin_yaw    = 0.0
        self._results: list[dict] = []

        # mission control
        self._start_event     = threading.Event()
        self._cancel_flag     = False
        self._active_handle   = None
        self._mission_running = False
        self._safe_lock       = False

        # ── subscribers ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, '/fix',               self._gps_cb,     10,
                                 callback_group=self._cb_group)
        self.create_subscription(Odometry,  '/odometry/filtered', self._odom_cb,    10,
                                 callback_group=self._cb_group)
        self.create_subscription(String,    '/mission_control',   self._mission_cb, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Joy,       '/joy',               self._joy_cb,     10,
                                 callback_group=self._cb_group)

        # ── publishers ─────────────────────────────────────────────────────
        self._photo_pub = self.create_publisher(String, '/take_photo', 10)

        # ── Nav2 ───────────────────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                 callback_group=self._cb_group)

        self.get_logger().info(
            'GPS Waypoint Follower ready.\n'
            f'  CSV: {self._csv_path}\n'
            '  Hold L1+L2, press X to START. Press O to CANCEL.\n'
            f'  Progress file: {PROGRESS_FILE}')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self._gps = msg

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_x   = msg.pose.pose.position.x
            self._odom_y   = msg.pose.pose.position.y
            self._odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)

    def _joy_cb(self, msg: Joy):
        # safety lock disabled — reserved for future use
        pass

    def _mission_cb(self, msg: String):
        if msg.data == 'START':
            if self._mission_running:
                self.get_logger().warn('Mission already running — ignoring START')
                return
            self.get_logger().info('START received.')
            self._cancel_flag = False
            self._results.clear()
            self._start_event.set()
        elif msg.data == 'CANCEL':
            if self._mission_running:
                self._do_cancel()

    def _do_cancel(self):
        self.get_logger().info('Mission cancelled.')
        self._cancel_flag = True
        with self._lock:
            handle = self._active_handle
        if handle is not None:
            handle.cancel_goal_async()

    # ── pose / coordinate helpers ──────────────────────────────────────────

    def _pose(self):
        with self._lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    def _ll_to_odom(self, lat, lon):
        """Flat-earth ENU: convert GPS lat/lon to odom-frame x/y relative to origin fix."""
        mpd_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(self._origin_lat))
        d_east  = (lon - self._origin_lon) * mpd_lon
        d_north = (lat - self._origin_lat) * _METERS_PER_DEG_LAT
        cos_y = math.cos(self._origin_yaw)
        sin_y = math.sin(self._origin_yaw)
        x = self._origin_x + cos_y * d_east + sin_y * d_north
        y = self._origin_y - sin_y * d_east + cos_y * d_north
        return x, y

    @staticmethod
    def _dist2d(x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def _make_pose(self, x, y, yaw=0.0):
        p = PoseStamped()
        p.header.frame_id = GOAL_FRAME
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    # ── progress persistence ───────────────────────────────────────────────

    def _load_progress(self) -> set:
        """Return set of already-completed waypoint names for this CSV."""
        try:
            with open(PROGRESS_FILE) as f:
                data = json.load(f)
            if data.get('csv') == self._csv_path:
                completed = set(data.get('completed', []))
                if completed:
                    self.get_logger().info(
                        f'Resuming — already completed: {sorted(completed)}')
                return completed
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        return set()

    def _save_progress(self, completed: set):
        with open(PROGRESS_FILE, 'w') as f:
            json.dump({'csv': self._csv_path, 'completed': list(completed)}, f)

    def _clear_progress(self):
        try:
            os.remove(PROGRESS_FILE)
        except FileNotFoundError:
            pass

    # ── Nav2 ───────────────────────────────────────────────────────────────

    def _navigate_to(self, x, y, yaw=0.0, label='goal') -> bool:
        if not self._nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        self.get_logger().info(
            f'  -> {label}  odom({x:.2f}, {y:.2f})  yaw={math.degrees(yaw):.0f}°')

        future = self._nav.send_goal_async(goal)
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

        status = result_future.result().status
        ok = status == 4
        self.get_logger().info(
            f'  {"Reached" if ok else "FAILED"} (status={status}): {label}'
            f'  [1=ACCEPTED 2=EXECUTING 4=SUCCEEDED 5=CANCELED 6=ABORTED]')
        return ok

    def _turn_and_photo(self, gx, gy, label: str):
        """Turn 90° right in place, take photo."""
        _, _, current_yaw = self._pose()
        right_yaw = current_yaw - math.pi / 2.0
        self.get_logger().info(f'  Turning right 90° for photo at {label}')
        self._navigate_to(gx, gy, yaw=right_yaw, label=f'{label}_turn')
        if not self._cancel_flag:
            msg = String()
            msg.data = label
            self._photo_pub.publish(msg)
            self.get_logger().info(f'  Photo taken: {label}')

    def _bypass_left(self, wp_x, wp_y, label: str):
        if not self._do_bypass or self._cancel_flag:
            return
        rx, ry, _ = self._pose()
        approach = math.atan2(wp_y - ry, wp_x - rx)
        left = approach + math.pi / 2.0
        bx = wp_x + math.cos(left) * self._lat_dist + math.cos(approach) * self._fwd_dist
        by = wp_y + math.sin(left) * self._lat_dist + math.sin(approach) * self._fwd_dist
        self.get_logger().info(f'  Bypassing {label} left -> ({bx:.2f}, {by:.2f})')
        self._navigate_to(bx, by, approach, label=f'{label}_bypass')

    # ── CSV ────────────────────────────────────────────────────────────────

    def _load_csv(self) -> list[tuple[str, float, float]]:
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
                    name = row[0].strip()
                    lat  = float(row[1])
                    lon  = float(row[2])
                    waypoints.append((name, lat, lon))
                except ValueError:
                    self.get_logger().warn(f'  Skipping bad row: {row}')
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from CSV.')
        return waypoints

    # ── startup waits ──────────────────────────────────────────────────────

    def _wait_gps(self, timeout=60.0):
        self.get_logger().info('Waiting for GPS fix...')
        deadline = time.time() + timeout
        last_log = 0
        while time.time() < deadline:
            time.sleep(0.5)
            if self._cancel_flag:
                return None
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
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.2)

    # ── mission ────────────────────────────────────────────────────────────

    def _run_mission(self):
        self._mission_running = True
        try:
            self.get_logger().info('Waiting for Nav2...')
            self._nav.wait_for_server()

            fix = self._wait_gps()
            if fix is None:
                return

            self._wait_odom()
            ox, oy, oyaw = self._pose()

            # Record startup GPS + odom as flat-earth origin
            with self._lock:
                self._origin_lat = fix.latitude
                self._origin_lon = fix.longitude
                self._origin_x   = ox
                self._origin_y   = oy
                self._origin_yaw = self._initial_heading

            self.get_logger().info(
                f'Origin: GPS({fix.latitude:.7f}, {fix.longitude:.7f})'
                f'  odom({ox:.3f}, {oy:.3f})  yaw={math.degrees(oyaw):.1f}°')

            waypoints = self._load_csv()
            if not waypoints:
                return

            # Convert GPS lat/lon → odom frame using flat-earth ENU
            self.get_logger().info('Converting waypoints GPS → odom frame...')
            goals = []
            for name, lat, lon in waypoints:
                x, y = self._ll_to_odom(lat, lon)
                goals.append((name, x, y))
                self.get_logger().info(f'  {name}: odom({x:.2f}, {y:.2f})')

            # load progress — skip already-visited waypoints
            completed = self._load_progress()

            for (name, lat, lon), (_, gx, gy) in zip(waypoints, goals):
                if self._cancel_flag:
                    break

                if name in completed:
                    self.get_logger().info(f'Skipping {name} (already completed)')
                    continue

                self.get_logger().info(f'\n{"─"*40}\nWaypoint: {name}\n{"─"*40}')
                reached = self._navigate_to(gx, gy, label=name)

                if reached and self._do_photo and not self._cancel_flag:
                    self._turn_and_photo(gx, gy, name)

                self._results.append({
                    'name': name, 'lat': lat, 'lon': lon,
                    'x': gx, 'y': gy, 'reached': reached})

                if reached:
                    completed.add(name)
                    self._save_progress(completed)

                self._bypass_left(gx, gy, name)

            if not self._cancel_flag:
                self.get_logger().info(f'\n{"─"*40}\nReturning home\n{"─"*40}')
                self._navigate_to(ox, oy, yaw=oyaw, label='home')
                self._clear_progress()   # mission complete — reset for next run
                self.get_logger().info('Progress cleared — ready for fresh run.')

            self._print_summary(waypoints, goals)
        finally:
            self._mission_running = False
            self._start_event.clear()

    def _print_summary(self, waypoints, goals):
        ok = sum(1 for r in self._results if r['reached'])
        status = 'CANCELLED' if self._cancel_flag else 'COMPLETE'
        lines = [
            '', '=' * 55,
            f'MISSION {status}',
            '=' * 55,
            f'  Waypoints : {ok}/{len(self._results)} reached',
            f'  Bypass    : {"ON" if self._do_bypass else "OFF"}',
            f'  Photo     : {"ON (right turn)" if self._do_photo else "OFF"}',
            '─' * 55,
        ]
        for (name, lat, lon), (_, gx, gy) in zip(waypoints, goals):
            r = next((r for r in self._results if r['name'] == name), None)
            if r is None:
                tick = 'SKIP'
            elif r['reached']:
                tick = 'OK  '
            else:
                tick = 'MISS'
            lines.append(
                f'  [{tick}] {name:20s}'
                f'  GPS({lat:.6f}, {lon:.6f})'
                f'  odom({gx:.1f}, {gy:.1f})')
        lines += ['─' * 55, '=' * 55]
        for line in lines:
            self.get_logger().info(line)

    # ── main loop ──────────────────────────────────────────────────────────

    def wait_and_run(self):
        while rclpy.ok():
            self.get_logger().info('Waiting for START (hold L1+L2, press X)...')
            self._start_event.wait()
            if not rclpy.ok():
                break
            self._run_mission()


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

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
