import launch
import os
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def get_car_auto_operation_steer_params(context):
    #path = LaunchConfiguration("/home/sit/autoware_erga/src/original_nodes/car_auto_operation/config/aaa.param.yaml").perform(context)
    path = "/home/sit/autoware_erga/src/original_nodes/car_auto_operation/config/aaa.param.yaml"
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    car_auto_operation_steer_params = get_car_auto_operation_steer_params(context)

    container = ComposableNodeContainer(
        name='containe_car_auto_operation',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            ComposableNode(#ステア自動運転ノード
                package='car_auto_operation',
                namespace='car_auto_operation',
                plugin='car_auto_operation::CarAutoOperationSteer',#ソースコード内で作成したノードクラス名を指定
                name='car_auto_operation_steer',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[car_auto_operation_steer_params],
            ),
        ],
        output='screen',
    )
    
    return[container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))


    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
