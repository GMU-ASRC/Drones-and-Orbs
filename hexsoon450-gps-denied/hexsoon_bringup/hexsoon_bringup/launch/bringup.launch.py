from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    vio_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_vio_launch'),
                'launch',
                'openvins_vio.launch.py'
            )
        )
    )

    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uav_dashboard'),
                'launch',
                'dashboard.launch.py'
            )
        )
    )

    test_runner_node = Node(
        package='flight_tests',
        executable='test_runner',
        name='flight_test_runner',
        output='screen',
    )

    uav_logger_node = Node(
        package='uav_logger',
        executable='logger_node',
        name='mission_logger',
        output='screen',
    )

    return LaunchDescription([
        #vio_stack,
        dashboard_launch,
        test_runner_node,
        uav_logger_node,
    ])
