from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        Node(
            package='traffic_signal_safety_guard',
            namespace='/perception/traffic_light_recognition',
            executable='traffic_signal_safety_guard_node',
            name='traffic_signal_safety_guard',
            parameters=[
                {'camera_info_timeout': 1.0},
                {'interval_time': 0.0667},
            ],
            remappings=[
                # Input
                ('/input/traffic_signals', '/perception/traffic_light_recognition/arbiter/traffic_signals'),
                ('/input/camera_info1', '/sensing/camera/camera6/camera_info'),
                ('/input/camera_info2', '/sensing/camera/camera7/camera_info'),
                # Output
                ('/output/traffic_signals', '/perception/traffic_light_recognition/traffic_signals'),
            ]
        )
    ])
