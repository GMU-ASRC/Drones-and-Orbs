from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    _launch_dir = os.path.join(
        get_package_share_directory('my_vio_launch'),
        'launch'
    )

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_launch_dir, 'map.launch.py')
        )
    )

    relay_launch = TimerAction(
        period=15.0,  # wait for RTAB-Map to initialise before relaying odom
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_launch_dir, 'relay.launch.py')
                )
            )
        ]
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hexsoon_bringup'),
                'launch',
                'bringup.launch.py'
            )
        )
    )

    return LaunchDescription([
        map_launch,
        # relay_launch,
    ])
