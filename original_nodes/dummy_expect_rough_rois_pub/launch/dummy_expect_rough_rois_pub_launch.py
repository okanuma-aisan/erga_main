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
            package='dummy_expect_rough_rois_pub',
            namespace='/perception/traffic_light_recognition/camera6',
            executable='dummy_expect_rough_rois_pub_node',
            name='dummy_expect_rough_rois_pub',
            parameters=[
                {'interval_time': 0.033},
                {'frame_id': "camera6_optical_link"},
                {'traffic_light_id': 2180},
                {'x_offset': 0},
                {'y_offset': 0},
                {'width': 2880},
                {'height': 1860},
            ],
            remappings=[
                # Output
                ('/output/expect_rois', '/perception/traffic_light_recognition/camera6/detection/expect/rois'),
                ('/output/rough_rois', '/perception/traffic_light_recognition/camera6/detection/rough/rois'),
                ('/output/debug/msg', '/perception/traffic_light_recognition/camera6/dummy/debug/msg'),
            ]
        ),
        Node(
            package='dummy_expect_rough_rois_pub',
            namespace='/perception/traffic_light_recognition/camera7',
            executable='dummy_expect_rough_rois_pub_node',
            name='dummy_expect_rough_rois_pub',
            parameters=[
                {'interval_time': 0.033},
                {'frame_id': "camera7_optical_link"},
                {'traffic_light_id': 2180},
                {'x_offset': 0},
                {'y_offset': 0},
                {'width': 2880},
                {'height': 1860},
            ],
            remappings=[
                # Output
                ('/output/expect_rois', '/perception/traffic_light_recognition/camera7/detection/expect/rois'),
                ('/output/rough_rois', '/perception/traffic_light_recognition/camera7/detection/rough/rois'),
                ('/output/debug/msg', '/perception/traffic_light_recognition/camera7/dummy/debug/msg'),
            ]
        )
    ])
