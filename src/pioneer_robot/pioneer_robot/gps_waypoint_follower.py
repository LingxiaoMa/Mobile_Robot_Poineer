import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# GPS waypoints for 3 of the 5 orange cones in basic_urdf.sdf
# ---------------------------------------------------------------------------
#
# Spherical origin (set in basic_urdf.sdf <spherical_coordinates>):
#   lat0 = -31.9805 deg,  lon0 = 115.8193 deg,  elevation = 20.0 m
#
# Gazebo ENU convention: world x = East, world y = North
#
# Forward conversion (SDF x/y → GPS) used to derive the values below:
#   METERS_PER_DEG_LAT = 111320.0
#   METERS_PER_DEG_LON = 111320.0 * cos(lat0)
#                      = 111320.0 * cos(-31.9805 * pi/180)
#                      = 111320.0 * 0.84811
#                      ≈ 94,444 m/deg
#
#   lat = lat0 + (y_m / METERS_PER_DEG_LAT)
#   lon = lon0 + (x_m / METERS_PER_DEG_LON)
#
# Inverse conversion (GPS → map x/y) used at runtime:
#   x_m = (lon - lon0) * METERS_PER_DEG_LON
#   y_m = (lat - lat0) * METERS_PER_DEG_LAT
#
# cone1  SDF pose (x= 3.0, y= 3.0):
#   lat = -31.9805 + ( 3.0 / 111320)  = -31.9804730
#   lon = 115.8193 + ( 3.0 /  94444)  = 115.8193318
#
# cone3  SDF pose (x= 6.0, y=-4.0):
#   lat = -31.9805 + (-4.0 / 111320)  = -31.9805359
#   lon = 115.8193 + ( 6.0 /  94444)  = 115.8193635
#
# cone5  SDF pose (x= 5.0, y= 6.0):
#   lat = -31.9805 + ( 6.0 / 111320)  = -31.9804461
#   lon = 115.8193 + ( 5.0 /  94444)  = 115.8193530
# ---------------------------------------------------------------------------
GPS_WAYPOINTS = [  # (latitude, longitude, label)
    (-31.9804730, 115.8193318, 'cone1'),
    (-31.9805359, 115.8193635, 'cone3'),
    (-31.9804461, 115.8193530, 'cone5'),
]

# Spherical origin — must match basic_urdf.sdf <spherical_coordinates>
_ORIGIN_LAT = -31.9805
_ORIGIN_LON = 115.8193
_METERS_PER_DEG_LAT = 111320.0
_METERS_PER_DEG_LON = 111320.0 * math.cos(math.radians(_ORIGIN_LAT))  # ≈ 94,444


def _ll_to_map(lat, lon):
    """Convert GPS lat/lon to map-frame x/y (flat-earth, ENU)."""
    x = (lon - _ORIGIN_LON) * _METERS_PER_DEG_LON
    y = (lat - _ORIGIN_LAT) * _METERS_PER_DEG_LAT
    return x, y


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 True
                             )
                         ])

        # GPS fix subscriber — sets flag + stores last fix for logging
        self._gps_fix_received = False
        self._last_fix = None
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_callback, 1
        )

        # FollowWaypoints: sends all poses to nav2_waypoint_follower in one call
        self._follow_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # NavigateToPose: used for the return-home leg
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF2 buffer to look up the robot's map-frame pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('GPS Waypoint Follower started.')

    # ------------------------------------------------------------------
    # GPS fix callback
    # ------------------------------------------------------------------

    def _gps_callback(self, msg):
        self._last_fix = msg
        if not self._gps_fix_received and msg.status.status >= 0:
            self._gps_fix_received = True

    # ------------------------------------------------------------------
    # Wait until /gps/fix reports a valid fix
    # ------------------------------------------------------------------

    def _wait_for_gps_fix(self, timeout=60.0):
        self.get_logger().info('Waiting for GPS fix...')
        start = time.time()          # wall clock — immune to sim time jumps
        last_log = -1
        while not self._gps_fix_received:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().error('Timed out waiting for GPS fix!')
                return False
            bucket = int(elapsed / 5) * 5
            if bucket > 0 and bucket != last_log:
                last_log = bucket
                self.get_logger().info(f'  still waiting for GPS fix... ({int(elapsed)}s)')
        fix = self._last_fix
        self.get_logger().info(
            f'GPS fix acquired  lat={fix.latitude:.7f}  lon={fix.longitude:.7f}  '
            f'status={fix.status.status}'
        )
        return True

    # ------------------------------------------------------------------
    # TF lookup: current robot pose in map frame
    # ------------------------------------------------------------------

    def _get_current_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose from TF: {e}')
            return None

    # ------------------------------------------------------------------
    # Convert a single lat/lon to a map-frame PoseStamped
    # Uses the inverse of the flat-earth formula from the comment block above.
    # ------------------------------------------------------------------

    def _convert_waypoint(self, lat, lon, label, index):
        total = len(GPS_WAYPOINTS)
        self.get_logger().info(
            f'Converting waypoint {index + 1}/{total} ({label}):  '
            f'lat={lat:.7f}  lon={lon:.7f}'
        )
        x, y = _ll_to_map(lat, lon)
        self.get_logger().info(f'  -> map frame: x={x:.3f}  y={y:.3f}')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0   # heading doesn't matter for cone visits
        return pose

    # ------------------------------------------------------------------
    # Send all waypoints to Nav2 FollowWaypoints action
    # ------------------------------------------------------------------

    def _follow_waypoints(self, poses, labels):
        self.get_logger().info(f'Sending {len(poses)} waypoints to Nav2...')

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        # feedback tells us which waypoint index is currently being navigated to
        def _feedback_cb(feedback_msg):
            idx = feedback_msg.feedback.current_waypoint
            if idx < len(labels):
                total = len(poses)
                self.get_logger().info(
                    f'  Navigating to waypoint {idx + 1}/{total} ({labels[idx]})...'
                )

        future = self._follow_client.send_goal_async(goal, feedback_callback=_feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error('FollowWaypoints goal was rejected by Nav2')
            return []

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        missed = list(result_future.result().result.missed_waypoints)
        reached = len(poses) - len(missed)
        self.get_logger().info(
            f'Nav2 complete: {reached}/{len(poses)} waypoints reached'
            + (f'  (missed indices: {missed})' if missed else '')
        )
        return missed

    # ------------------------------------------------------------------
    # Drive back to home pose via NavigateToPose
    # ------------------------------------------------------------------

    def _navigate_home(self, home_pose):
        x = home_pose.pose.position.x
        y = home_pose.pose.position.y
        self.get_logger().info(f'Returning to start: x={x:.2f}  y={y:.2f}')

        goal = NavigateToPose.Goal()
        goal.pose = home_pose

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error('NavigateToPose (home) goal was rejected')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code
        if error_code == 0:
            self.get_logger().info('Home reached. Mission complete.')
        else:
            self.get_logger().warn(f'Could not reach home (error_code={error_code})')

    # ------------------------------------------------------------------
    # Main orchestration
    # ------------------------------------------------------------------

    def run(self):
        # 1. Wait for Nav2 action servers
        self.get_logger().info('Waiting for Nav2 action servers...')
        self._follow_client.wait_for_server()
        self._nav_client.wait_for_server()
        self.get_logger().info('  Nav2 servers ready.')

        # 2. Wait for a valid GPS fix before doing anything
        if not self._wait_for_gps_fix():
            return

        # 3. Record home pose (map frame) before moving anywhere
        self.get_logger().info('Recording home position...')
        home_pose = None
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.5)
            home_pose = self._get_current_map_pose()
            if home_pose is not None:
                break

        if home_pose is None:
            self.get_logger().error('Could not record home pose from TF — aborting.')
            return

        hx = home_pose.pose.position.x
        hy = home_pose.pose.position.y
        self.get_logger().info(f'Recorded home position: x={hx:.2f}  y={hy:.2f}')

        # 4. Convert GPS waypoints to map-frame poses
        poses = []
        labels = []
        for i, (lat, lon, label) in enumerate(GPS_WAYPOINTS):
            pose = self._convert_waypoint(lat, lon, label, i)
            poses.append(pose)
            labels.append(label)

        # 5. Send all converted poses to Nav2 FollowWaypoints
        self._follow_waypoints(poses, labels)

        # 6. Return home
        self._navigate_home(home_pose)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
