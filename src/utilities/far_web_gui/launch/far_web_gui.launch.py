"""
Launch file for FAR Web GUI:
  - rosbridge_websocket (WebSocket bridge for roslibjs)
  - graph_file_bridge   (VGH serialization + HTTP file server)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.conditions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ws_port',   default_value='9090',
                              description='rosbridge WebSocket port'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='HTTP port for web GUI static files'),
        DeclareLaunchArgument(
            'suppress_nonessential_topics',
            default_value='true',
            description='If true, rosbridge only allows topics/services required by FAR Web GUI.'
        ),

        # rosbridge WebSocket server (filtered)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('ws_port'),
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
                # Strict allow-list for FAR Web GUI traffic
                'topics_glob': "['/state_estimation','/far_reach_goal_status','/viz_graph_topic','/viz_path_topic','/viz_node_topic','/goal_point','/joy','/planning_attemptable','/update_visibility_graph','/reset_visibility_graph','/web_gui/upload_graph']",
                'services_glob': "['/web_gui/download_graph','/rosapi/*']",
            }],
            condition=launch.conditions.IfCondition(LaunchConfiguration('suppress_nonessential_topics')),
        ),

        # rosbridge WebSocket server (unfiltered fallback)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('ws_port'),
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
            }],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('suppress_nonessential_topics')),
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
