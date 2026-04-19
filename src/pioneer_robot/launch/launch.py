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

    return LaunchDescription([

        # DDS
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value='rmw_cyclonedds_cpp'
        ),

        # 1. Pioneer robot driver
        # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        # ros2 run ariaNode ariaNode -rp /dev/ttyUSB0
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='aria_node',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),

        # 2. SICK lidar
        # ros2 run sick_scan_xd sick_generic_caller --ros-args \
        #   -p scanner_type:=sick_tim_7xx \
        #   -p hostname:=192.168.0.1 \
        #   -p use_binary_protocol:=true \
        #   --remap sick_tim_7xx/scan:=/scan
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
        # ros2 run joy joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 4. Main controller
        # ros2 run pioneer_robot joy_controller
        Node(
            package='pioneer_robot',
            executable='joy_controller',
            name='joy_controller',
            output='screen'
        ),

        # 5. OAK-D driver
        # ros2 run pioneer_robot oak_driver_node
        # topics:
        # /oak/rgb/image_raw
        # /oak/stereo/image_raw
        Node(
            package='pioneer_robot',
            executable='oak_driver_node',
            name='oak_driver_node',
            output='screen'
        ),

        # 6. IMU
        # ros2 launch phidgets_spatial spatial-launch.py
        # topic:
        # /imu/data_raw
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phidgets_launch)
        ),

        # 7. GPS
        # ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
        # topic:
        # /fix
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

        # 8. Local controller
        # ros2 run pioneer_robot local_controller
        Node(
            package='pioneer_robot',
            executable='local_controller',
            name='local_controller',
            output='screen',
            parameters=[{'use_dwa': False}]
        ),

        # 9. Cone detector
        # ros2 run pioneer_robot cone_detector
        Node(
            package='pioneer_robot',
            executable='cone_detector',
            name='cone_detector',
            output='screen'
        ),
    ])