from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    phidgets_launch = os.path.join(
        get_package_share_directory('phidgets_spatial'),
        'launch',
        'spatial-launch.py'
    )

    nav2_params = os.path.join(
        get_package_share_directory('pioneer_robot'),
        'config',
        'nav2_params.yaml'
    )

    ekf_params = os.path.join(
        get_package_share_directory('pioneer_robot'),
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([

        # DDS
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value='rmw_cyclonedds_cpp'
        ),

        # 1. Pioneer robot driver
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='aria_node',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),

        # 2. SICK lidar
        # tf_base_frame_id: cloud 挂在 base_link 下（默认是 map，会产生孤立树）
        # tf_base_lidar_xyz_rpy: 安装位置 + yaw=-1.5708 修正 90° 安装偏转
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_tim_7xx',
            output='screen',
            parameters=[{
                'scanner_type': 'sick_tim_7xx',
                'hostname': '192.168.0.1',
                'use_binary_protocol': True,
                'tf_base_frame_id': 'base_link',
                'tf_base_lidar_xyz_rpy': '0.20 0.0 0.104 0.0 0.0 -1.5708',
            }],
            remappings=[('sick_tim_7xx/scan', '/scan')]
        ),

        # 3. Joystick
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),

        # 4. Joystick controller (teleop override)
        Node(
            package='pioneer_robot',
            executable='joy_controller',
            name='joy_controller',
            output='screen'
        ),

        # 5. OAK-D driver
        Node(
            package='pioneer_robot',
            executable='oak_driver_node',
            name='oak_driver_node',
            output='screen'
        ),

        # 6. IMU — publishes /imu/data_raw
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phidgets_launch)
        ),

        # 7. GPS
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyACM0', 'baud': 9600}]
        ),

        # 8. EKF — /odom + /imu/data_raw → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),

        # 9. Nav2 — 直接启动各节点，绕开 nav2_bringup 的 lifecycle_nodes 硬编码
        #    (nav2_bringup/navigation_launch.py 在 Jazzy 中强制包含 docking_server/route_server，
        #     这两个节点无法 configure 会卡住整个 autostart 链)
        Node(package='nav2_controller',   executable='controller_server',  name='controller_server',  output='screen', parameters=[nav2_params]),
        Node(package='nav2_smoother',     executable='smoother_server',    name='smoother_server',    output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner',      executable='planner_server',     name='planner_server',     output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors',    executable='behavior_server',    name='behavior_server',    output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',       name='bt_navigator',       output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[nav2_params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=[nav2_params]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen', parameters=[nav2_params]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 4.0,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),

        # 10. Goal relay — /move_relative (Point) → NavigateToPose action
        Node(
            package='pioneer_robot',
            executable='goal_relay',
            name='goal_relay',
            output='screen',
        ),

        # 11. Cone detector
        Node(
            package='pioneer_robot',
            executable='cone_detector',
            name='cone_detector',
            output='screen'
        ),

        # 12. Object detector (colour + shape + distance for unknown obstacles)
        Node(
            package='pioneer_robot',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
    ])
