"""
Launch file for FAR Web GUI:
  - rosbridge_websocket (WebSocket bridge for roslibjs)
  - graph_file_bridge   (VGH serialization + HTTP file server)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ws_port',   default_value='9090',
                              description='rosbridge WebSocket port'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='HTTP port for web GUI static files'),

        # rosbridge WebSocket server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
            }],
        ),

        # rosapi (needed by roslibjs for service calls, topic listing, etc.)
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
        ),

        # Graph file bridge + HTTP server
        Node(
            package='far_web_gui',
            executable='graph_file_bridge.py',
            name='graph_file_bridge',
            output='screen',
            parameters=[{
                'http_port': LaunchConfiguration('http_port'),
            }],
        ),
    ])
