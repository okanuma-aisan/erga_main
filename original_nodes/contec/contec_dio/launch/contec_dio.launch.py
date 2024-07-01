import launch
import os
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def get_params():
    path = os.path.join(
        get_package_share_directory('wada_vmc'),
        'config',
        'wada_vmc_erga.param.yaml'
        )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()

    container = ComposableNodeContainer(
        name='containe_contec_dio',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            #ペダル関連
            ComposableNode(#contecから車両方法を取得するノード
                package='contec_dio',
                #namespace='contec',
                plugin='contec::ContecDio',#ソースコード内で作成したノードクラス名を指定
                name='contec_dio',
                extra_arguments=[{"use_intra_process_comms": False}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[
                    {'device_name': LaunchConfiguration('contec_device_name')},#contecデバイスID
                    {'can_update_time_th_sec': LaunchConfiguration('can_update_time_th_sec')},#canからの通信がこの時間を過ぎたらクラッチを切る
                ],
                remappings=[
                    ("pedal_info", LaunchConfiguration('pedal_info')),
                    ("steer_info", LaunchConfiguration('steer_info')),
                    ("auto_permission", LaunchConfiguration('auto_permission')),
                ],
            ),
         ],
    )
    
    return[container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("contec_device_name", "DIO000")#contecデバイスID
    add_launch_arg("can_update_time_th_sec", "0.3")#canからの通信がこの時間を過ぎたらクラッチを切る
    add_launch_arg("pedal_info", "/vehicle/wada_vmc/pedal/can501")#pedal
    add_launch_arg("steer_info", "/vehicle/wada_vmc/steer/can502")#steer
    add_launch_arg("auto_permission", "/vehicle/auto_permission/auto_permission")
    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
