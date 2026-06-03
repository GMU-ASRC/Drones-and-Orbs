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
    rtabmap_examples = get_package_share_directory('rtabmap_examples')

    # ── Launch Arguments ───────────────────────────────────────────────────────
    use_web_video_server_arg = DeclareLaunchArgument(
        'use_web_video_server',
        default_value='true',
        description='Launch web_video_server for browser-based stream viewing'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        # default_value='serial:///dev/ttyACM0:115200',
        default_value='udp://127.0.0.1:5761@',
        description='FCU connection URL for OrangeCube'
    )

    # ── UAV Dashboard ─────────────────────────────────────────────────────────
    # NOTE: temporary — included here for convenience during development.
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uav_dashboard'),
                'launch',
                'dashboard.launch.py'
            )
        )
    )

    # ── RealSense D435i ────────────────────────────────────────────────────────
    mapping_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(rtabmap_examples, 'launch', 'realsense_d435i_infra.launch.py')
    ),
        launch_arguments={
            'unite_imu_method': '2',   # linear_interpolation — fixes dropped IMU
            'gyro_fps':         '200', # pin rate; don't inherit rtabmap_examples defaults
            'accel_fps':        '200',
        }.items()
    )

    # ── MAVROS ─────────────────────────────────────────────────────────────────
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
        }],
    )

    # ── Pose type converter ────────────────────────────────────────────────────
    # OpenVINS: PoseWithCovarianceStamped → MAVROS vision_pose: PoseStamped
    pose_converter_node = Node(
        package='topic_tools',
        executable='transform',
        name='pose_converter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            '/odom',                        # input
            '/mavros/vision_pose/pose',                 # output
            'geometry_msgs/msg/PoseStamped',            # output type
            'geometry_msgs.msg.PoseStamped('
            'header=m.header,'
            'pose=m.pose.pose)',                        # strip covariance wrapper
            '--import', 'geometry_msgs',
            '--qos-reliability', 'reliable',
        ]
    )

    # ── Odometry relay ─────────────────────────────────────────────────────────
    # OpenVINS: Odometry → MAVROS odometry/in (same type, relay only)
    odometry_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odometry_relay',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            '/odom',        # input
            '/mavros/odometry/in',       # output
        ]
    )

    # ── Web Video Server ───────────────────────────────────────────────────────
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='log',
        parameters=[{'port': 8080}],
        condition=IfCondition(LaunchConfiguration('use_web_video_server'))
    )

    # ── Optional RViz ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        #use_web_video_server_arg,
        # fcu_url_arg,
        #realsense_node,
        # mapping_node,
        # mavros_node,
        # web_video_server_node,
        pose_converter_node,
        # dashboard_launch,
        odometry_relay_node, 
        # rviz_node,   # uncomment to enable RViz
    ])
