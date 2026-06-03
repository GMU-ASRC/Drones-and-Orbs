from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────────
    config_path = os.path.join(
        get_package_share_directory('my_vio_launch'),
        'config',
        'estimator_config.yaml'
    )
    realsense_share = get_package_share_directory('realsense2_camera')

    # ── RealSense D435i ────────────────────────────────────────────────────────
    realsense_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(realsense_share, 'launch', 'rs_launch.py')
    ),
        launch_arguments={
            'unite_imu_method': '1',
            'enable_gyro':      'true',
            'enable_accel':     'true',
            'gyro_fps':		'200',
            'accel_fps':	'200',
            # ── Stereo IR cameras (global shutter, HW-synced to IMU) ──
            'enable_infra1':    'true',
            'enable_infra2':    'true',
            'infra_fps':        '90',
            'infra_width':      '640',
            'infra_height':     '480',
            'enable_sync':	'true',
            # ── Disable emitter so IR images are texture-rich ──────────
            # (emitter projects a dot pattern that corrupts feature tracking)
            'emitter_enabled':  '0',
            # ── Disable color & depth (not needed for VIO) ─────────────
            'enable_color':     'true',
            'color_width':      '640',
            'color_height':     '480',
            'depth_width':      '640',
            'depth_height':     '480',            
            'enable_depth':     'true',
            'publish_tf':	'true'
        }.items()
    )

    # ── Optional RViz ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch',
                'display_ros2.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        #rviz_node,   # uncomment to enable RViz
    ])
