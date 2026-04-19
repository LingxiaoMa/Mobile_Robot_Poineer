import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('pioneer_robot')
    
    urdf_path = os.path.join(pkg_dir, 'resources', 'robots', 'pioneer.urdf')
    world_path = os.path.join(pkg_dir, 'resources', 'worlds', 'basic_urdf.sdf')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    navsat_params_path = os.path.join(pkg_dir, 'config', 'navsat_params.yaml')
    teleop_config_path = os.path.join(pkg_dir, 'config', 'teleop_config.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        #launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),
        #spawn robot after delay
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'service', '-s', '/world/pioneer_world/create',
                        '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000',
                        '--req', f'sdf_filename: "{urdf_path}", name: "pioneer"'],
                    output='screen'
                ),
            ]
        ),

        #robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        #ros gz Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ]
        ),

        #joystick driver — reads /dev/input/js0, publishes /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0, 'use_sim_time': True}]
        ),

        #converts /joy → /cmd_vel  (hold L1 to drive, L1+R1 for turbo)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config_path, {'use_sim_time': True}]
        ),

        #navsat transform (provides /fromLL service for GPS -> map coordinate conversion)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_params_path, {'use_sim_time': True}],
            remappings=[
                ('imu', '/imu'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odom'),
            ]
        ),

        #slam tooloolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_params.yaml')
            }.items()
        ),


        #controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        #smoother Server
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params_path]
        ),

        #planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path]
        ),

        #behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        #BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path]
        ),

        #waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_path]
        ),

        #velocity smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_path],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        #collision Monitor
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[nav2_params_path]
        ),

        #lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ]
            }]
        ),

    ])