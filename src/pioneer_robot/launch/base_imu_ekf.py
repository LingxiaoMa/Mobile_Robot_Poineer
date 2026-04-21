from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ekf_params = os.path.join(
        get_package_share_directory('pioneer_robot'),
        'config',
        'ekf_params.yaml'
    )

    nav2_params = os.path.join(
        get_package_share_directory('pioneer_robot'),
        'config',
        'nav2_simple_params.yaml'
    )

    return LaunchDescription([

        # 1. Pioneer chassis
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='aria_node',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),

        # 2. Static TF: base_link → imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_imu_static',
            output='screen',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 3. Static TF: map → odom (identity, required by bt_navigator)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_static',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 4. Phidgets IMU
        ComposableNodeContainer(
            name='phidget_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    plugin='phidgets::SpatialRosI',
                    name='phidgets_spatial',
                    parameters=[{
                        'use_orientation': True,
                        'spatial_algorithm': 'ahrs',
                        'frame_id': 'imu_link',
                        'ahrs_angular_velocity_threshold': 1.0,
                        'ahrs_angular_velocity_delta_threshold': 0.1,
                        'ahrs_acceleration_threshold': 0.1,
                        'ahrs_mag_time': 10.0,
                        'ahrs_accel_time': 10.0,
                        'ahrs_bias_time': 1.25,
                        'heating_enabled': False,
                    }],
                ),
            ],
            output='both',
        ),

        # 5. EKF: /odom + /imu/data_raw → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),

        # 6. Nav2 — mapless, odom frame, straight-line planner, no obstacle layer
        Node(package='nav2_controller',   executable='controller_server',  name='controller_server',  output='screen', parameters=[nav2_params]),
        Node(package='nav2_smoother',     executable='smoother_server',    name='smoother_server',    output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner',      executable='planner_server',     name='planner_server',     output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors',    executable='behavior_server',    name='behavior_server',    output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',       name='bt_navigator',       output='screen', parameters=[nav2_params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=[nav2_params]),
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
                    'velocity_smoother',
                ],
            }],
        ),

    ])
