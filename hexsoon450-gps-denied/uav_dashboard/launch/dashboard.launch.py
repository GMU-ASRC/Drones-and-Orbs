from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    http_server_node = Node(
        package='uav_dashboard',
        executable='http_server',
        name='dashboard_http_server',
        output='screen',
        parameters=[{
            'port':    8000,
            'ws_port': 9090,
        }]
    )

    return LaunchDescription([
        http_server_node,
    ])
