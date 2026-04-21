#!/usr/bin/env python3
"""
gps_logger.py — Continuously prints the robot's current GPS coordinates.

Subscribes to /fix (NavSatFix) and prints lat/lon at 1 Hz.
Also shows the filtered odom position from /odometry/filtered.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GpsLogger(Node):

    def __init__(self):
        super().__init__('gps_logger')
        self.declare_parameter('print_hz', 1.0)
        hz = self.get_parameter('print_hz').value

        self._fix = None
        self._odom_x = None
        self._odom_y = None

        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_timer(1.0 / hz, self._print_cb)

        self.get_logger().info(f'GPS Logger started — printing at {hz:.1f} Hz')

    def _gps_cb(self, msg: NavSatFix):
        self._fix = msg

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _print_cb(self):
        if self._fix is None:
            self.get_logger().info('Waiting for /fix...')
            return

        fix = self._fix
        status_str = 'FIX' if fix.status.status >= 0 else 'NO_FIX'
        odom_str = (f'  odom({self._odom_x:.2f}, {self._odom_y:.2f})'
                    if self._odom_x is not None else '')
        self.get_logger().info(
            f'GPS [{status_str}]  lat={fix.latitude:.7f}  lon={fix.longitude:.7f}{odom_str}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
