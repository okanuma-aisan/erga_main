from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 起動したいノードを記述
        Node(
            package='valgrind_sample',
            executable='valgrind_sample',
            prefix=['valgrind --tool=callgrind'],
        ),
    ])
