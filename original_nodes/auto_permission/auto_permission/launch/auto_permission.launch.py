import launch
import os
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def get_params():
    path = os.path.join(
        get_package_share_directory('auto_permission'),
        'config',
        'auto_permission_param.yaml'
        )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["autoware_permission"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()

    container = ComposableNodeContainer(
        name='containe_auto_permission',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            ComposableNode(#自動運転許可ノード
                package='auto_permission',
                namespace='perm',
                plugin='auto_permission::AutoPermission',#ソースコード内で作成したノードクラス名を指定
                name='auto_permission',
                extra_arguments=[{"use_intra_process_comms": False}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[params],
                remappings=[
                    ("gnss_dev", "/sensing/gnss/novatel_oem7/pos_var"),
                    ("nearest_voxel_transformation_likelihood", "/localization/pose_estimator/nearest_voxel_transformation_likelihood"),
                    ("transform_probability", "/localization/pose_estimator/transform_probability"),
                    ("curr_pos", "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias"),
                    ("curr_vel", "/localization/pose_twist_fusion_filter/twist_with_covariance"),
                    ("scenario_trajectry", "/planning/scenario_planning/trajectory"),
                    ("report_select", "/report_selector/report_select")
                    ("pedal_info_20221111", "/vehicle/pedal/can501"),
                    ("steer_info_20221111", "/vehicle/steer/can502"),
                    ("can_info_20210103", "/vehicle/steer/can502"),
                ],
            ),
        ],
    )

    return[container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
