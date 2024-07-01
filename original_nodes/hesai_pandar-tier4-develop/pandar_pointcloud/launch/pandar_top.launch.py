import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('pcap', '')
    add_launch_arg('device_ip', '192.168.2.200')
    add_launch_arg('lidar_port', '2368')
    add_launch_arg('gps_port', '10110')
    add_launch_arg('scan_phase', '0.0')
    add_launch_arg('return_mode', 'Dual')
    add_launch_arg('dual_return_distance_threshold', '0.1')
    add_launch_arg('model', 'Pandar64')
    add_launch_arg('frame_id', 'lidar_top')
    add_launch_arg('calibration', '$(find-pkg-share pandar_pointcloud)/config/64.csv')
    add_launch_arg('use_intra_process', 'True')
    add_launch_arg('distance_range', '[0.1, 200.0]')

    nodes = []

    nodes.append(
        ComposableNode(#
            package='pandar_driver',
            plugin='pandar_driver::PandarDriver',#ソースコード内で作成したノードクラス名を指定
            name='pandar_driver',
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
            parameters=[
                {'pcap': LaunchConfiguration('pcap')},
                {'device_ip': LaunchConfiguration('device_ip')},
                {'lidar_port': LaunchConfiguration('lidar_port')},
                {'gps_port': LaunchConfiguration('gps_port')},
                {'scan_phase': LaunchConfiguration('scan_phase')},
                {'model': LaunchConfiguration('model')},
                {'frame_id': LaunchConfiguration('frame_id')},
            ],
        ),
    )

    nodes.append(
        ComposableNode(#
            package='pandar_pointcloud',
            plugin='pandar_pointcloud::PandarCloud',#ソースコード内で作成したノードクラス名を指定
            name='pandar_pointcloud',
            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
            parameters=[
                {'scan_phase': LaunchConfiguration('scan_phase')},
                {'return_mode': LaunchConfiguration('return_mode')},
                {'dual_return_distance_threshold': LaunchConfiguration('dual_return_distance_threshold')},
                {'model': LaunchConfiguration('model')},
                {'device_ip': LaunchConfiguration('device_ip')},
                {'calibration': LaunchConfiguration('calibration')},
                {'distance_range': LaunchConfiguration('distance_range')}
            ],
            remappings=[
                ("pandar_points", "pointcloud_raw"),
                ("pandar_points_ex", "pointcloud_raw_ex"),
            ],
        ),
    )
    
    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "pointcloud_raw_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    container = ComposableNodeContainer(
        name="pandar_node_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
    )

    return launch.LaunchDescription(launch_arguments + [container])
    
