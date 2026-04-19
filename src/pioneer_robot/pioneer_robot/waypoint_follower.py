import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import subprocess
import shutil
import os


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower_node',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 True
                             )
                         ])
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.recording_process = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._map_received = False
        self._current_map = None
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 1
        )
        self.get_logger().info('Waypoint Follower Node started!')

    def _map_callback(self, msg):
        self._current_map = msg
        if not self._map_received:
            free_cells = sum(1 for c in msg.data if c == 0)
            if free_cells > 500:
                self._map_received = True
                self.get_logger().info(
                    f'Map ready: {msg.info.width}x{msg.info.height} '
                    f'({free_cells} free cells)'
                )

#checking where robot is in map frame, based of described dimensions (drivetrain) can calc global coords
    def _log_robot_pose(self, label):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.get_logger().info(
                f'{label} -> x={pos.x:.2f}m  y={pos.y:.2f}m  yaw={math.degrees(yaw):.1f}deg'
            )
        except Exception as e:
            self.get_logger().warn(f'could not get robot pose: {e}')


    #aborting issue with slams 'moving' map, goal is now fluidish...
    def find_nearest_free_cell(self, x, y, search_radius=3.0):
        """scan the grid outward"""
        if self._current_map is None:
            return x, y

        info = self._current_map.info

        def cell_value(cx, cy):
            if 0 <= cx < info.width and 0 <= cy < info.height:
                return self._current_map.data[cy * info.width + cx]
            return -1  # unknown 

        #calc coords into grid position
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)

        #goal cell itself is free then no change needed
        if cell_value(mx, my) == 0:
            return x, y

        #mgoal coords lethal value, move outward to find the nearest free cell
        max_steps = int(search_radius / info.resolution)
        for r in range(1, max_steps + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:  # nly check the perimeter
                        cx, cy = mx + dx, my + dy
                        if cell_value(cx, cy) == 0:
                            wx = info.origin.position.x + (cx + 0.5) * info.resolution
                            wy = info.origin.position.y + (cy + 0.5) * info.resolution
                            return wx, wy

        #if still not possible will error and abort waypoint
        self.get_logger().warn(
            f'no free cell found within {search_radius}m of ({x:.1f},{y:.1f})  using original'
        )
        return x, y

    def navigate_to(self, pose, index):
        x_orig = pose.pose.position.x
        y_orig = pose.pose.position.y

        #adjust goal to nearest free cell if occupied, calling function above
        adj_x, adj_y = self.find_nearest_free_cell(x_orig, y_orig)
        if abs(adj_x - x_orig) > 0.05 or abs(adj_y - y_orig) > 0.05:
            self.get_logger().warn(
                f'WP{index} ({x_orig:.1f},{y_orig:.1f}) is occupied in map '
                f'adjusted to ({adj_x:.2f},{adj_y:.2f})'
            )
            pose.pose.position.x = adj_x
            pose.pose.position.y = adj_y

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error(f'WP{index}: goal rejected by Nav2')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code
        if error_code == 0:
            self._log_robot_pose(f'Arrived at WP{index}')
            return True
        else:
            self.get_logger().warn(
                f'WP{index}: could not reach ({adj_x:.1f},{adj_y:.1f}), '
                f'error_code={error_code} continuing to next waypoint'
            )
            self._log_robot_pose(f'Skipped WP{index}, current position')
            return False

#automated recording stuff
    def start_recording(self):
        self.get_logger().info('Starting bag recording...')
        bag_path = os.path.expanduser(
            '~/ros2_ws/src/pioneer_robot/resources/maps/path_recording'
        )
        if os.path.exists(bag_path):
            shutil.rmtree(bag_path)
        self.recording_process = subprocess.Popen([
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '/odom', '/scan', '/tf', '/cmd_vel'
        ])

    def stop_recording(self):
        if self.recording_process:
            self.get_logger().info('Stopping bag recording...')
            self.recording_process.terminate()
            self.recording_process.wait()
            self.get_logger().info('Recording saved!')

    def save_map(self):
        self.get_logger().info('savng map')
        map_path = os.path.expanduser(
            '~/ros2_ws/src/pioneer_robot/resources/maps/map'
        )
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path
        ])
        self.get_logger().info('map saved')

    #for nav to work slam needs to have mapped some area. this checks if thats true
    def wait_for_map(self, timeout_sec=120.0):
        self.get_logger().info('Waiting for SLAM map to be ready...')
        start = self.get_clock().now()
        last_log = 0
        while not self._map_received:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    'Timed out waiting for map!'
                )
                return False
            if int(elapsed) % 10 == 0 and int(elapsed) > 0 and int(elapsed) != last_log:
                last_log = int(elapsed)
                self.get_logger().info(
                    f'Still waiting for map... ({int(elapsed)}s). '
                )
        return True

#cycling thru set waypoints
    def send_waypoints(self, waypoints):
        self.get_logger().info('waiting for Nav2')
        self._nav_client.wait_for_server()

        if not self.wait_for_map():
            return

        self.get_logger().info(f'starting navigation  {len(waypoints)} waypoints')
        self.start_recording()

        completed = 0
        for i, pose in enumerate(waypoints):
            self.get_logger().info(
                f' WP{i}: ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})'
            )
            if self.navigate_to(pose, i): #error handling if a waypoint cannot be pathed too it will skip to next
                completed += 1

        self.get_logger().info(
            f'complete: {completed}/{len(waypoints)} waypoints reached'
        )
        self.stop_recording()
        self.save_map()

#setting up coord system
def create_waypoint(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(args=None):
    rclpy.init(args=args)

    node = WaypointFollower()

    waypoints = [
        create_waypoint(  0.0,  5.0,  0.0),            
        create_waypoint(  9.0, -1.0,  math.pi),       
        create_waypoint(  0.0, -6.0,  math.pi),        
        create_waypoint( -6.0,  1.0,  math.pi / 2),   
        create_waypoint(  0.0,  0.0,  0.0),            
    ]

    node.send_waypoints(waypoints)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
