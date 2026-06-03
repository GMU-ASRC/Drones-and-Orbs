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

    # ── Launch Arguments ───────────────────────────────────────────────────────
    use_web_video_server_arg = DeclareLaunchArgument(
        'use_web_video_server',
        default_value='true',
        description='Launch web_video_server for browser-based stream viewing'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:921600',
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
    realsense_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(realsense_share, 'launch', 'rs_launch.py')
    ),
        launch_arguments=list({
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
            'rgb_width':      '640',
            'rgb_height':     '480',
            'enable_depth':     'true',
            'publish_tf':	'true'
        }.items())
    )

    # ── OpenVINS ───────────────────────────────────────────────────────────────
    ov_msckf_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='ov_msckf',
        output='screen',
        parameters=[{
            'config_path':      config_path,
            'verbosity':        'INFO',
            'use_stereo':       True,   # ← was False
            'max_cameras':      2,      # ← was 1
            'save_total_state': False,
        }],
        remappings=[
            # IR left  → cam0
            ('/cam0/image_raw', '/camera/camera/infra1/image_rect_raw'),
            # IR right → cam1  ← new
            ('/cam1/image_raw', '/camera/camera/infra2/image_rect_raw'),
            ('/imu0',           '/camera/camera/imu'),
        ]
    )

    # ── MAVROS ─────────────────────────────────────────────────────────────────
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'plugin_blacklist': ['imu']
        }],
    )

    # ── Pose type converter ────────────────────────────────────────────────────
    # OpenVINS: PoseWithCovarianceStamped → MAVROS vision_pose: PoseStamped
    pose_converter_node = Node(
        package='topic_tools',
        executable='transform',
        name='pose_converter',
        output='screen',
        arguments=[
            '/ov_msckf/poseimu',                        # input
            '/mavros/vision_pose/pose',                 # output
            'geometry_msgs/msg/PoseStamped',            # output type
            'geometry_msgs.msg.PoseStamped('
            'header=m.header,'
            'pose=m.pose.pose)',                        # strip covariance wrapper
            '--import', 'geometry_msgs',
        ]
    )

    # ── Odometry relay ─────────────────────────────────────────────────────────
    # OpenVINS: Odometry → MAVROS odometry/in (same type, relay only)
    odometry_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odometry_relay',
        output='screen',
        arguments=[
            '/ov_msckf/odomimu',        # input
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
        use_web_video_server_arg,
        fcu_url_arg,
        realsense_node,
        ov_msckf_node,
        mavros_node,
        web_video_server_node,
        pose_converter_node,
        odometry_relay_node, 
        # dashboard_launch,
        # rviz_node,   # uncomment to enable RViz
    ])
