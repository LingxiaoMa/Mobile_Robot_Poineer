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

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
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
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_tim_7xx',
            output='screen',
            parameters=[{
                'scanner_type': 'sick_tim_7xx',
                'hostname': '192.168.0.1',
                'use_binary_protocol': True,
            }],
            remappings=[
                ('sick_tim_7xx/scan', '/scan')
            ]
        ),

        # 3. Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

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
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 9600,
            }]
        ),

        # 8. EKF — fuses /odom + /imu/data_raw → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),

        # 9. Nav2 navigation stack
        #    Global planner : StraightLine (no map required)
        #    Local controller: RegulatedPurePursuit + lidar costmap
        #    cmd_vel pipeline: controller → velocity_smoother → collision_monitor → /cmd_vel
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
            }.items()
        ),

        # 10. Goal relay — converts /move_relative (Point) → NavigateToPose action
        #     x, y = metres relative to robot start position
        #     z    = final yaw offset in degrees (0 = keep start heading)
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
    ])
