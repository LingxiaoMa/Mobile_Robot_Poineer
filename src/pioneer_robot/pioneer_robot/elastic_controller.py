#!/usr/bin/env python3
"""
Pioneer Elastic Controller — Pure Pursuit + A* + Potential Field obstacle avoidance

Replaces DWA with a potential-field repulsion approach:
  - Obstacle positions are stored in a time-decaying memory (global frame)
  - A repulsion vector from the nearest obstacle is blended with the goal heading
  - Emergency backup triggers when an obstacle enters the front blind zone

Subscribes:
  /goal_pose          geometry_msgs/PoseStamped
  /odometry/filtered  nav_msgs/Odometry
  /scan               sensor_msgs/LaserScan
  /map                nav_msgs/OccupancyGrid

Publishes:
  /cmd_vel_auto       geometry_msgs/Twist
  /followed_path      nav_msgs/Path
"""

import math
import heapq
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ──────────────────────────────────────────────────────────────────────────────
# A* on OccupancyGrid
# ──────────────────────────────────────────────────────────────────────────────

def _inflate(grid: np.ndarray, radius: int) -> np.ndarray:
    inflated = grid.copy()
    rows, cols = np.where(grid)
    for r, c in zip(rows.tolist(), cols.tolist()):
        r0, r1 = max(0, r - radius), min(grid.shape[0], r + radius + 1)
        c0, c1 = max(0, c - radius), min(grid.shape[1], c + radius + 1)
        inflated[r0:r1, c0:c1] = True
    return inflated


def astar(grid: np.ndarray,
          start: tuple[int, int],
          goal:  tuple[int, int]) -> list[tuple[int, int]] | None:
    rows, cols = grid.shape

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: list = []
    heapq.heappush(open_heap, (h(start, goal), start))
    came_from: dict = {}
    g: dict = {start: 0.0}

    DIRS  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    COSTS = [1.0,   1.0,  1.0,  1.0,  1.414,  1.414, 1.414, 1.414]

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]

        for (dr, dc), cost in zip(DIRS, COSTS):
            nb = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nb[0] < rows and 0 <= nb[1] < cols):
                continue
            if grid[nb[0], nb[1]]:
                continue
            ng = g[cur] + cost
            if ng < g.get(nb, float('inf')):
                came_from[nb] = cur
                g[nb] = ng
                heapq.heappush(open_heap, (ng + h(nb, goal), nb))

    return None


def _prune(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0 = pruned[-1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if abs(cross) > 0.05:
            pruned.append(path[i])
    pruned.append(path[-1])
    return pruned


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class ElasticController(Node):

    IDLE              = 0
    FOLLOWING_PATH    = 1
    ROTATE_TO_HEADING = 2

    def __init__(self):
        super().__init__('elastic_controller')

        # Base navigation parameters
        self.declare_parameter('lookahead_dist',  0.5)
        self.declare_parameter('max_linear_vel',  0.25)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('kp_angular',      1.5)
        self.declare_parameter('xy_tolerance',    0.20)
        self.declare_parameter('yaw_tolerance',   0.05)
        self.declare_parameter('inflate_radius',  3)      # A* obstacle inflation, cells

        # Potential field parameters (replaces DWA)
        self.declare_parameter('pf_inflation_radius',   0.25)  # robot body margin (m)
        self.declare_parameter('pf_safe_dist',          1.2)   # repulsion starts at this dist
        self.declare_parameter('pf_repel_gain',         2.5)   # repulsion strength
        self.declare_parameter('pf_memory_duration',    4.0)   # obstacle memory TTL (s)
        self.declare_parameter('pf_turn_in_place_thresh', 0.7) # rad — turn in place above this
        self.declare_parameter('pf_scan_max_range',     3.0)   # ignore scan beyond this (m)

        # State
        self.state   = self.IDLE
        self.cx = self.cy = self.cyaw = 0.0
        self.odom_ok = False
        self.path: list[tuple[float, float]] = []
        self.goal_x = self.goal_y = self.goal_yaw = 0.0
        self._goal_pose_msg: PoseStamped | None = None
        self._last_replan_time = 0.0

        # Obstacle memory: list of (global_x, global_y, timestamp)
        self._obstacle_memory: list[tuple[float, float, float]] = []
        self._last_avoid_turn = 0.8   # sign used for emergency backup spin

        # Laser scan (raw, sensor frame)
        self._scan_ranges:   list[float] = []
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0

        # Map
        self._map: OccupancyGrid | None = None

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ROS interfaces
        self.create_subscription(Odometry,      '/odometry/filtered', self._odom_cb,  10)
        self.create_subscription(PoseStamped,   '/goal_pose',         self._goal_cb,  10)
        self.create_subscription(LaserScan,     '/scan',              self._scan_cb,  10)
        self.create_subscription(OccupancyGrid, '/map',               self._map_cb,   10)

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel_auto',  10)
        self.path_pub = self.create_publisher(Path,  '/followed_path', 10)

        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            'ElasticController ready (A* + Pure Pursuit + Potential Field).\n'
            '  Waiting for /goal_pose …')

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _scan_cb(self, msg: LaserScan):
        self._scan_ranges    = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_inc = msg.angle_increment

        if not self.odom_ok:
            return

        # Project scan points into global frame and store in obstacle memory
        scan_max     = self.get_parameter('pf_scan_max_range').value
        current_time = time.time()

        for i in range(0, len(self._scan_ranges), 3):   # subsample every 3rd beam
            r = self._scan_ranges[i]
            if math.isfinite(r) and 0.05 < r < scan_max:
                a          = self._scan_angle_min + i * self._scan_angle_inc
                glob_angle = normalize_angle(self.cyaw + a)
                obs_x      = self.cx + r * math.cos(glob_angle)
                obs_y      = self.cy + r * math.sin(glob_angle)
                self._obstacle_memory.append((obs_x, obs_y, current_time))

        # Expire old obstacle points
        ttl = self.get_parameter('pf_memory_duration').value
        self._obstacle_memory = [
            pt for pt in self._obstacle_memory
            if current_time - pt[2] < ttl
        ]

    def _odom_cb(self, msg: Odometry):
        self.cx   = msg.pose.pose.position.x
        self.cy   = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        self.cyaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_ok = True

    def _goal_cb(self, msg: PoseStamped):
        if not self.odom_ok:
            self.get_logger().warn('No odometry yet, ignoring goal.')
            return

        src_frame = msg.header.frame_id or 'odom'
        if src_frame != 'odom':
            try:
                tf = self._tf_buffer.lookup_transform(
                    'odom', src_frame, rclpy.time.Time())
                transformed = PoseStamped()
                transformed.header.frame_id = 'odom'
                transformed.pose = do_transform_pose(msg.pose, tf)
                msg = transformed
            except Exception as e:
                self.get_logger().warn(
                    f'TF lookup failed ({src_frame}→odom): {e}. Using as-is.')

        self.goal_x   = msg.pose.position.x
        self.goal_y   = msg.pose.position.y
        self.goal_yaw = quat_to_yaw(*[getattr(msg.pose.orientation, a)
                                      for a in ('x', 'y', 'z', 'w')])
        self._goal_pose_msg = msg
        self.get_logger().info(
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'{math.degrees(self.goal_yaw):.1f}°) — planning...')
        self._stop()
        self._plan_and_start()

    # ──────────────────────────────────────────────────────────────────
    # A* path planning
    # ──────────────────────────────────────────────────────────────────

    def _plan_and_start(self):
        path = self._plan_path_astar(self.cx, self.cy, self.goal_x, self.goal_y)
        if path is None:
            self.get_logger().error('A*: no path found!')
            self.state = self.IDLE
            return
        self.path = path
        self._last_replan_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = self.FOLLOWING_PATH
        self.get_logger().info(f'A* path: {len(self.path)} waypoints.')
        self._publish_path_viz()

    def _plan_path_astar(self, sx, sy, gx, gy) -> list[tuple[float, float]] | None:
        if self._map is None:
            self.get_logger().warn('No map available, using straight-line path.')
            return [(sx, sy), (gx, gy)]

        m   = self._map
        res = m.info.resolution
        ox  = m.info.origin.position.x
        oy  = m.info.origin.position.y
        W   = m.info.width
        H   = m.info.height

        data     = np.array(m.data, dtype=np.int8).reshape((H, W))
        obstacle = (data > 50)
        inflate_r = self.get_parameter('inflate_radius').value
        obstacle  = _inflate(obstacle, inflate_r)

        def to_cell(wx, wy):
            return (int((wy - oy) / res), int((wx - ox) / res))

        def to_world(r, c):
            return (c * res + ox + res * 0.5, r * res + oy + res * 0.5)

        def clamp_cell(rc):
            return (max(0, min(H - 1, rc[0])), max(0, min(W - 1, rc[1])))

        start_cell = clamp_cell(to_cell(sx, sy))
        goal_cell  = clamp_cell(to_cell(gx, gy))
        obstacle[start_cell] = False
        obstacle[goal_cell]  = False

        grid_path = astar(obstacle, start_cell, goal_cell)
        if grid_path is None:
            return None

        return _prune([to_world(r, c) for r, c in grid_path])

    def _publish_path_viz(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        for wx, wy in self.path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    # ──────────────────────────────────────────────────────────────────
    # Control loop (20 Hz)
    # ──────────────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.odom_ok:
            return
        if self.state == self.FOLLOWING_PATH:
            self._pure_pursuit()
        elif self.state == self.ROTATE_TO_HEADING:
            self._align_heading()

    # ──────────────────────────────────────────────────────────────────
    # Pure Pursuit with Potential Field blending
    # ──────────────────────────────────────────────────────────────────

    def _pure_pursuit(self):
        if not self.path:
            self.state = self.IDLE
            return

        L       = self.get_parameter('lookahead_dist').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        xy_tol  = self.get_parameter('xy_tolerance').value
        turn_thresh   = self.get_parameter('pf_turn_in_place_thresh').value
        inflation_r   = self.get_parameter('pf_inflation_radius').value

        # Drop path points behind the robot
        while len(self.path) > 1:
            dx = self.path[0][0] - self.cx
            dy = self.path[0][1] - self.cy
            local_x = math.cos(-self.cyaw) * dx - math.sin(-self.cyaw) * dy
            if local_x < 0.0:
                self.path.pop(0)
            else:
                break

        # Check arrival
        if math.hypot(self.goal_x - self.cx, self.goal_y - self.cy) < xy_tol:
            self._stop()
            self.state = self.ROTATE_TO_HEADING
            self.get_logger().info('Position reached — aligning heading...')
            return

        lookahead = self._find_lookahead(L)

        goal_angle  = math.atan2(lookahead[1] - self.cy, lookahead[0] - self.cx)
        heading_err = normalize_angle(goal_angle - self.cyaw)

        # Blend goal heading with potential field repulsion
        avoid_x, avoid_y = self._pf_avoidance()
        if abs(avoid_x) > 0.01 or abs(avoid_y) > 0.01:
            v_x = math.cos(heading_err) + avoid_x
            v_y = math.sin(heading_err) + avoid_y
            heading_err = math.atan2(v_y, v_x)

        # Emergency backup: obstacle entered the robot's front blind zone
        if self._scan_ranges:
            raw_min = min(self._scan_ranges)
            min_idx = self._scan_ranges.index(raw_min)
            num_beams = len(self._scan_ranges)
            center_zone = (num_beams * 0.3 < min_idx < num_beams * 0.7)
            if (raw_min - inflation_r) < 0.05 and center_zone:
                twist = Twist()
                twist.linear.x  = -0.15
                twist.angular.z = self._last_avoid_turn
                self.cmd_pub.publish(twist)
                return
            # Update escape spin direction based on which side is closer
            if raw_min < 1.5:
                self._last_avoid_turn = 0.8 if min_idx > num_beams / 2 else -0.8

        # Turn in place when heading error is too large
        if abs(heading_err) > turn_thresh:
            twist = Twist()
            twist.angular.z = max(-max_ang, min(max_ang, 2.0 * heading_err))
            self.cmd_pub.publish(twist)
            return

        # Normal forward motion with speed scaled by obstacle proximity
        avoid_magnitude = math.hypot(avoid_x, avoid_y)
        speed_factor = max(0.4, 1.0 - avoid_magnitude * 0.4)
        lin_vel = (max_lin * speed_factor
                   * max(0.3, 1.0 - 0.6 * min(abs(heading_err) / (math.pi / 2), 1.0)))
        ang_vel = max(-max_ang, min(max_ang, 2.5 * heading_err))

        twist = Twist()
        twist.linear.x  = lin_vel
        twist.angular.z = ang_vel
        self.cmd_pub.publish(twist)

    # ──────────────────────────────────────────────────────────────────
    # Potential field obstacle avoidance
    # ──────────────────────────────────────────────────────────────────

    def _pf_avoidance(self) -> tuple[float, float]:
        """
        Returns a (avoid_x, avoid_y) repulsion vector in the robot's local frame.
        The vector combines a perpendicular deflection (70%) and a pure repulsion (30%).
        """
        if not self._obstacle_memory:
            return 0.0, 0.0

        inflation_r = self.get_parameter('pf_inflation_radius').value
        safe_dist   = self.get_parameter('pf_safe_dist').value
        repel_gain  = self.get_parameter('pf_repel_gain').value

        # Find closest obstacle point
        min_dist     = 999.0
        closest_obs  = None
        for pt in self._obstacle_memory:
            d = math.hypot(pt[0] - self.cx, pt[1] - self.cy)
            if d < min_dist:
                min_dist    = d
                closest_obs = pt

        if closest_obs is None:
            return 0.0, 0.0

        eff_dist = min_dist - inflation_r
        if eff_dist >= safe_dist:
            return 0.0, 0.0

        # Relative angle to obstacle in robot frame
        angle_to_obs = math.atan2(closest_obs[1] - self.cy, closest_obs[0] - self.cx)
        rel_angle    = normalize_angle(angle_to_obs - self.cyaw)

        eff_dist = max(0.01, eff_dist)
        strength = ((safe_dist - eff_dist) / safe_dist) * repel_gain

        # Perpendicular deflection direction (steer away to the side)
        avoid_ang = rel_angle - math.pi / 2.0 if rel_angle >= 0 else rel_angle + math.pi / 2.0
        repel_ang = normalize_angle(rel_angle + math.pi)

        avoid_x = strength * (0.7 * math.cos(avoid_ang) + 0.3 * math.cos(repel_ang))
        avoid_y = strength * (0.7 * math.sin(avoid_ang) + 0.3 * math.sin(repel_ang))

        return avoid_x, avoid_y

    def _find_lookahead(self, L):
        for px, py in self.path:
            if math.hypot(px - self.cx, py - self.cy) >= L:
                return (px, py)
        return self.path[-1]

    # ──────────────────────────────────────────────────────────────────
    # Final heading alignment
    # ──────────────────────────────────────────────────────────────────

    def _align_heading(self):
        max_ang = self.get_parameter('max_angular_vel').value
        kp_ang  = self.get_parameter('kp_angular').value
        yaw_tol = self.get_parameter('yaw_tolerance').value

        yaw_err = normalize_angle(self.goal_yaw - self.cyaw)
        if abs(yaw_err) < yaw_tol:
            self._stop()
            self.state = self.IDLE
            self.get_logger().info('Goal reached (position + heading).')
            return

        twist = Twist()
        twist.angular.z = max(-max_ang, min(max_ang, kp_ang * yaw_err))
        self.cmd_pub.publish(twist)

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ElasticController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
