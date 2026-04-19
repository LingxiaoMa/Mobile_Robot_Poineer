#!/usr/bin/env python3
"""
Pioneer Local Controller — Pure Pursuit + custom A*

Architecture:
  1. Receives a goal from /goal_pose  (RViz "2D Nav Goal" button)
  2. Plans a path using A* on the /map occupancy grid (no Nav2 planner)
  3. Follows the path using Pure Pursuit
  4. Uses DWA for local obstacle avoidance
  5. Aligns to the final heading once at the goal position
  6. Publishes /cmd_vel

Subscribes:
  /goal_pose      geometry_msgs/PoseStamped
  /odom           nav_msgs/Odometry
  /scan           sensor_msgs/LaserScan
  /map            nav_msgs/OccupancyGrid

Publishes:
  /cmd_vel        geometry_msgs/Twist
  /followed_path  nav_msgs/Path
"""

import math
import heapq
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


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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
    """Dilate obstacle cells by `radius` cells."""
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
    """
    A* on a boolean grid (True = obstacle).
    Returns list of (row, col) from start to goal, or None if no path.
    """
    rows, cols = grid.shape

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: list = []
    heapq.heappush(open_heap, (h(start, goal), start))
    came_from: dict = {}
    g: dict = {start: 0.0}

    DIRS = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    COSTS = [1.0,  1.0,  1.0,  1.0,  1.414,  1.414, 1.414, 1.414]

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
    """Remove collinear waypoints to reduce path length."""
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

class LocalController(Node):

    IDLE             = 0
    FOLLOWING_PATH   = 1
    ROTATE_TO_HEADING = 2

    def __init__(self):
        super().__init__('local_controller')

        # Parameters
        self.declare_parameter('lookahead_dist',       0.5)
        self.declare_parameter('max_linear_vel',       0.25)
        self.declare_parameter('max_angular_vel',      0.8)
        self.declare_parameter('kp_angular',           1.5)
        self.declare_parameter('xy_tolerance',         0.20)
        self.declare_parameter('yaw_tolerance',        0.05)
        self.declare_parameter('replan_interval',      2.0)
        self.declare_parameter('inflate_radius',       3)    # cells
        self.declare_parameter('use_dwa',              True) # set False to disable local avoidance

        # DWA parameters
        self.declare_parameter('dwa_w_samples',        20)
        self.declare_parameter('dwa_sim_time',         2.0)   # was 1.0
        self.declare_parameter('dwa_sim_step',         0.1)
        self.declare_parameter('dwa_min_obstacle_dist',0.2)   # Pioneer radius ~0.25m + margin
        self.declare_parameter('dwa_w_heading',        1.0)
        self.declare_parameter('dwa_w_obstacle',       2.0)
        self.declare_parameter('dwa_scan_max_range',   3.0)

        # State
        self.state   = self.IDLE
        self.cx = self.cy = self.cyaw = 0.0
        self.odom_ok = False
        self._prev_w = 0.0   # last angular command — for oscillation suppression
        self._osc_count = 0  # consecutive sign-flip counter for oscillation detection
        self._escape_ticks = 0  # backup escape countdown
        self.path: list[tuple[float, float]] = []
        self.goal_x = self.goal_y = self.goal_yaw = 0.0
        self._goal_pose_msg: PoseStamped | None = None
        self._last_replan_time = 0.0

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Map
        self._map: OccupancyGrid | None = None

        # Laser scan
        self._scan_ranges:    list[float] = []
        self._scan_angle_min  = 0.0
        self._scan_angle_inc  = 0.0

        # ROS interfaces
        self.create_subscription(Odometry,       '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(PoseStamped,    '/goal_pose', self._goal_cb, 10)
        self.create_subscription(LaserScan,      '/scan',      self._scan_cb, 10)
        self.create_subscription(OccupancyGrid,  '/map',       self._map_cb,  10)

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel_auto',  10)
        self.path_pub = self.create_publisher(Path,  '/followed_path', 10)

        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            'LocalController ready (A* + Pure Pursuit + DWA).\n'
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

    def _odom_cb(self, msg: Odometry):
        self.cx  = msg.pose.pose.position.x
        self.cy  = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.cyaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_ok = True

    def _goal_cb(self, msg: PoseStamped):
        if not self.odom_ok:
            self.get_logger().warn('No odometry yet, ignoring goal.')
            return

        # Transform goal into odom frame if needed
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
                self.get_logger().warn(f'TF lookup failed ({src_frame}→odom): {e}. Using as-is.')

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quat_to_yaw(*[getattr(msg.pose.orientation, a)
                                      for a in ('x','y','z','w')])
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

    def _plan_path_astar(self, sx, sy, gx, gy) -> list[tuple[float,float]] | None:
        if self._map is None:
            # No map yet — straight line fallback
            self.get_logger().warn('No map available, using straight-line path.')
            return [(sx, sy), (gx, gy)]

        m   = self._map
        res = m.info.resolution
        ox  = m.info.origin.position.x
        oy  = m.info.origin.position.y
        W   = m.info.width
        H   = m.info.height

        # Build boolean obstacle grid from OccupancyGrid
        data = np.array(m.data, dtype=np.int8).reshape((H, W))
        obstacle = (data > 50)          # occupied
        # treat unknown (-1) as free
        inflate_r = self.get_parameter('inflate_radius').value
        obstacle = _inflate(obstacle, inflate_r)

        def to_cell(wx, wy):
            c = int((wx - ox) / res)
            r = int((wy - oy) / res)
            return (r, c)

        def to_world(r, c):
            return (c * res + ox + res * 0.5,
                    r * res + oy + res * 0.5)

        start_cell = to_cell(sx, sy)
        goal_cell  = to_cell(gx, gy)

        # Clamp to grid bounds
        def clamp_cell(rc):
            return (max(0, min(H-1, rc[0])),
                    max(0, min(W-1, rc[1])))

        start_cell = clamp_cell(start_cell)
        goal_cell  = clamp_cell(goal_cell)

        # Clear start/goal cells so we don't block ourselves
        obstacle[start_cell] = False
        obstacle[goal_cell]  = False

        grid_path = astar(obstacle, start_cell, goal_cell)
        if grid_path is None:
            return None

        # Convert to world and prune collinear points
        world_path = [to_world(r, c) for r, c in grid_path]
        return _prune(world_path)

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
    # Pure Pursuit
    # ──────────────────────────────────────────────────────────────────

    def _pure_pursuit(self):
        if not self.path:
            self.state = self.IDLE
            return

        # Backup escape mode — back up for N ticks then replan
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            twist = Twist()
            twist.linear.x = -0.1
            self.cmd_pub.publish(twist)
            if self._escape_ticks == 0:
                self._plan_and_start()
            return

        L       = self.get_parameter('lookahead_dist').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        xy_tol  = self.get_parameter('xy_tolerance').value

        # Drop path points already behind the robot
        while len(self.path) > 1:
            dx = self.path[0][0] - self.cx
            dy = self.path[0][1] - self.cy
            local_x = math.cos(-self.cyaw) * dx - math.sin(-self.cyaw) * dy
            if local_x < 0.0:
                self.path.pop(0)
            else:
                break

        # Check arrival
        goal_dist = math.hypot(self.goal_x - self.cx, self.goal_y - self.cy)
        if goal_dist < xy_tol:
            self._stop()
            self.state = self.ROTATE_TO_HEADING
            self.get_logger().info('Position reached — aligning heading...')
            return

        lookahead = self._find_lookahead(L)

        if self.get_parameter('use_dwa').value:
            linear_vel, angular_vel = self._dwa_control(
                lookahead[0], lookahead[1], max_lin, max_ang)
        else:
            goal_angle = math.atan2(lookahead[1] - self.cy, lookahead[0] - self.cx)
            heading_err = normalize_angle(goal_angle - self.cyaw)
            kp_ang = self.get_parameter('kp_angular').value
            linear_vel  = max_lin * max(0.3, 1.0 - 0.6 * min(abs(heading_err) / (math.pi / 2), 1.0))
            angular_vel = max(-max_ang, min(max_ang, kp_ang * heading_err))

        twist = Twist()
        twist.linear.x  = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

    # ──────────────────────────────────────────────────────────────────
    # DWA
    # ──────────────────────────────────────────────────────────────────

    def _dwa_control(self, goal_x, goal_y, max_lin, max_ang):
        w_samples = self.get_parameter('dwa_w_samples').value
        sim_time  = self.get_parameter('dwa_sim_time').value
        sim_step  = self.get_parameter('dwa_sim_step').value
        min_obs   = self.get_parameter('dwa_min_obstacle_dist').value
        wh        = self.get_parameter('dwa_w_heading').value
        wo        = self.get_parameter('dwa_w_obstacle').value
        scan_max  = self.get_parameter('dwa_scan_max_range').value

        obs: list[tuple[float, float]] = []
        for i, r in enumerate(self._scan_ranges):
            if math.isfinite(r) and 0.05 < r < scan_max:
                a = self._scan_angle_min + i * self._scan_angle_inc
                obs.append((r * math.cos(a), r * math.sin(a)))

        n_steps = max(1, int(sim_time / sim_step))
        goal_angle_robot = normalize_angle(
            math.atan2(goal_y - self.cy, goal_x - self.cx) - self.cyaw)

        # Sample both v and w (real DWA)
        v_candidates = [max_lin, max_lin * 0.6, max_lin * 0.3]

        best_score = -1e9
        best_v, best_w = 0.0, 0.0

        for v in v_candidates:
            for wi in range(w_samples + 1):
                w = -max_ang + 2.0 * max_ang * wi / w_samples
                rx, ry, ryaw = 0.0, 0.0, 0.0
                min_clearance = 1e9
                collision = False

                for _ in range(n_steps):
                    rx   += v * math.cos(ryaw) * sim_step
                    ry   += v * math.sin(ryaw) * sim_step
                    ryaw += w * sim_step
                    for ox, oy in obs:
                        d = math.hypot(rx - ox, ry - oy)
                        if d < min_obs:
                            collision = True
                            break
                        if d < min_clearance:
                            min_clearance = d
                    if collision:
                        break

                if collision:
                    continue

                traj_angle    = math.atan2(ry, rx) if (rx != 0 or ry != 0) else 0.0
                heading_score = math.cos(normalize_angle(goal_angle_robot - traj_angle))
                obstacle_score = (min(min_clearance, 2.0) / 2.0
                                  if min_clearance < 1e8 else 1.0)
                velocity_score = v / max_lin
                # Penalise direction flips to suppress left-right oscillation
                continuity_score = 1.0 - abs(w - self._prev_w) / (2.0 * max_ang)
                score = (wh * heading_score
                         + wo * obstacle_score
                         + 0.3 * velocity_score
                         + 1.5 * continuity_score)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        # No safe trajectory found — rotate toward most open space
        if best_score <= -1e9:
            open_w = self._most_open_direction(max_ang)
            # Oscillation detection: if we keep flipping direction, trigger backup
            if self._prev_w != 0.0 and (open_w * self._prev_w < 0):
                self._osc_count += 1
            else:
                self._osc_count = 0
            if self._osc_count >= 4:
                self._osc_count = 0
                self._escape_ticks = 10  # ~0.5 s backup at 20 Hz
                self.get_logger().warn('Oscillation detected — backing up to escape.')
            self._prev_w = open_w
            return 0.0, open_w

        # Oscillation detection for normal DWA output
        if self._prev_w != 0.0 and (best_w * self._prev_w < 0):
            self._osc_count += 1
        else:
            self._osc_count = 0
        if self._osc_count >= 6:
            self._osc_count = 0
            self._escape_ticks = 10
            self.get_logger().warn('Oscillation detected — backing up to escape.')

        self._prev_w = best_w
        return best_v, best_w

    def _most_open_direction(self, max_ang: float) -> float:
        """Return angular velocity toward the most open (longest range) scan direction."""
        if not self._scan_ranges:
            return max_ang * 0.5
        best_idx = max(range(len(self._scan_ranges)),
                       key=lambda i: self._scan_ranges[i] if math.isfinite(self._scan_ranges[i]) else 0.0)
        best_angle = self._scan_angle_min + best_idx * self._scan_angle_inc
        return max_ang * 0.5 * (1.0 if best_angle > 0 else -1.0)

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
    node = LocalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
