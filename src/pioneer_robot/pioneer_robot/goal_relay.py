#!/usr/bin/env python3
"""
goal_relay — relative goal → Nav2 NavigateToPose action

Subscribes:
  /move_relative  geometry_msgs/Point
    x, y  = displacement in metres from the robot's position at startup
    z     = desired final yaw in degrees (0 = forward, ignored if z == 999)

On first odometry message the node records the robot's starting pose.
Each /move_relative message sends a NavigateToPose goal in the odom frame:
  goal = start_pose + (x, y)

Usage examples (from any terminal with ROS2 sourced):
  # Move 2 m forward
  ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"

  # Move 2 m forward, 1 m left, face 90° left on arrival
  ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 2.0, y: 1.0, z: 90.0}"
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalRelay(Node):

    def __init__(self):
        super().__init__('goal_relay')

        self._start_x:   float | None = None
        self._start_y:   float | None = None
        self._start_yaw: float | None = None

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(Point,    '/move_relative',     self._goal_cb, 10)

        self.get_logger().info(
            'goal_relay ready.\n'
            '  Waiting for first odometry to record start pose…\n'
            '  Then publish to /move_relative (geometry_msgs/Point):\n'
            '    x, y = metres from start   z = final yaw in degrees')

    def _odom_cb(self, msg: Odometry):
        if self._start_x is None:
            self._start_x   = msg.pose.pose.position.x
            self._start_y   = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self._start_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'Start pose recorded: '
                f'({self._start_x:.3f}, {self._start_y:.3f}, '
                f'{math.degrees(self._start_yaw):.1f}°)')

    def _goal_cb(self, msg: Point):
        if self._start_x is None:
            self.get_logger().warn('No odometry yet — ignoring goal.')
            return

        # Rotate relative displacement by start yaw so that
        # x=1 always means "1 m forward from start heading"
        cos_y = math.cos(self._start_yaw)
        sin_y = math.sin(self._start_yaw)
        abs_x = self._start_x + cos_y * msg.x - sin_y * msg.y
        abs_y = self._start_y + sin_y * msg.x + cos_y * msg.y

        if msg.z == 999.0:
            goal_yaw = self._start_yaw
        else:
            goal_yaw = self._start_yaw + math.radians(msg.z)

        qx, qy, qz, qw = yaw_to_quat(goal_yaw)

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = abs_x
        goal_msg.pose.pose.position.y = abs_y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal: relative ({msg.x:.2f}, {msg.y:.2f}) → '
            f'odom ({abs_x:.2f}, {abs_y:.2f}), yaw {math.degrees(goal_yaw):.1f}°')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            return
        self.get_logger().info('Goal accepted.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(f'  distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def _result_cb(self, future):
        result = future.result()
        if result.status == 4:   # SUCCEEDED
            self.get_logger().info('Goal reached.')
        else:
            self.get_logger().warn(f'Goal ended with status {result.status}.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
