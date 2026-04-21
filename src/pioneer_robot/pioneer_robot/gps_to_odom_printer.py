#!/usr/bin/env python3
"""
gps_to_odom_printer.py

Reads GPS waypoints from a CSV file, waits for the first valid /fix,
then prints each waypoint's odom-frame coordinates to the terminal.

Assumes robot starts facing north (initial_heading_deg = 90.0).

CSV format:
    # name, latitude, longitude
    cone_1, -31.9804730, 115.8193318

Usage:
    ros2 run pioneer_robot gps_to_odom_printer
    ros2 run pioneer_robot gps_to_odom_printer --ros-args -p initial_heading_deg:=90.0
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory

_METERS_PER_DEG_LAT = 111320.0


class GpsToOdomPrinter(Node):

    def __init__(self):
        super().__init__('gps_to_odom_printer')

        default_csv = os.path.join(
            get_package_share_directory('pioneer_robot'), 'config', 'waypoints.csv')
        self.declare_parameter('waypoints_file',      default_csv)
        self.declare_parameter('initial_heading_deg', 90.0)  # 90=north

        self._csv_path = self.get_parameter('waypoints_file').value
        self._heading  = math.radians(self.get_parameter('initial_heading_deg').value)

        self._origin_fix = None

        self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)
        self.get_logger().info(f'Waiting for GPS fix...  (CSV: {self._csv_path})')

    def _fix_cb(self, msg: NavSatFix):
        if self._origin_fix is not None:
            return
        if msg.status.status < 0 or math.isnan(msg.latitude):
            return

        self._origin_fix = msg
        self.get_logger().info(
            f'Origin fix: lat={msg.latitude:.7f}  lon={msg.longitude:.7f}')
        self._print_waypoints()

    def _ll_to_odom(self, lat, lon):
        mpd_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(self._origin_fix.latitude))
        d_east  = (lon - self._origin_fix.longitude) * mpd_lon
        d_north = (lat - self._origin_fix.latitude)  * _METERS_PER_DEG_LAT
        cos_h = math.cos(self._heading)
        sin_h = math.sin(self._heading)
        x =  cos_h * d_east + sin_h * d_north
        y = -sin_h * d_east + cos_h * d_north
        return x, y

    def _load_csv(self):
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
                    waypoints.append((row[0].strip(), float(row[1]), float(row[2])))
                except ValueError:
                    self.get_logger().warn(f'Skipping bad row: {row}')
        return waypoints

    def _print_waypoints(self):
        waypoints = self._load_csv()
        if not waypoints:
            return

        heading_deg = math.degrees(self._heading)
        print()
        print('=' * 60)
        print(f'  Origin : lat={self._origin_fix.latitude:.7f}  lon={self._origin_fix.longitude:.7f}')
        print(f'  Heading: {heading_deg:.1f}° (90=north 0=east)')
        print('=' * 60)
        print(f'  {"Name":<20} {"lat":>12} {"lon":>13}   {"odom_x":>8} {"odom_y":>8}  {"dist":>7}')
        print('-' * 60)
        for name, lat, lon in waypoints:
            x, y = self._ll_to_odom(lat, lon)
            dist = math.hypot(x, y)
            print(f'  {name:<20} {lat:>12.7f} {lon:>13.7f}   {x:>8.2f} {y:>8.2f}  {dist:>6.1f}m')
        print('=' * 60)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdomPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
