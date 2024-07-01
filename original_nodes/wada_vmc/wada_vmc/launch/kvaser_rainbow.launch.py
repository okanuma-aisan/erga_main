import launch
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
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
        'wada_vmc_rainbow.param.yaml'
        )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()

    nodes=[
        Node(#joyboardからデータを受信するノード
            package='wada_vmc',
            namespace='',
            executable='kvaser_receiver',
            parameters=[
                {'serial1': LaunchConfiguration('serial1')},
                {'serial2': LaunchConfiguration('serial2')},
                {'wait_time_ms': LaunchConfiguration('wait_time_ms')},#kvaserのシリアル番号及びタイムアウト設定
            ],
        ),
        Node(#受信したデータをmessage id毎にトピック化するノード
            package='wada_vmc',
            namespace='',
            executable='receive_to_topic_20210103',
            parameters=[{'select_can_device': 'all'}] + [params],
            remappings=[
                ("steering_status", "/vehicle/status/steering_status"),
            ]
        ),
        Node(#操舵数値をjoy boardに送信するノード
            package='wada_vmc',
            namespace='',
            executable='kvaser_sender_20210103',
            parameters=[
                {'serial1': LaunchConfiguration('serial1')},
                {'serial2': LaunchConfiguration('serial2')},
            ],
        ),
        
        #自動運転計算
        Node(#ステア自動運転ノード
            package='car_auto_operation',
            namespace='',
            executable='car_auto_operation_steer',
            parameters=[params],
            remappings=[
                ("autoware_steer_cmd", "/control/command/control_cmd"),
            ],
        ),
        Node(#autowareの情報からcanの操舵数値を計算するノード
            package='wada_vmc',
            namespace='',
            executable='vmc_calculator_20210103',
            parameters=[params],
            remappings=[
                ("auto_permission", "/vehicle/auto_permission/overlay_error_text"),
            ],
        ),
    ]
    
    return nodes

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('serial1', '77264')#ペダルkvaserのシリアル番号１番目
    add_launch_arg('serial2', '0')#ペダルkvaserのシリアル番号２番め
    add_launch_arg('wait_time_ms', '10')#kvaser受信のウェイトタイム(ミリ秒)

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
