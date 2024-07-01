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
        'wada_vmc_erga.param.yaml'
        )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()

    nodes=[
        #ペダル関連
        Node(#joyboardからデータを受信するノード
            package='wada_vmc',
            namespace='pedal',
            executable='kvaser_receiver',
            parameters=[
                {'serial1': LaunchConfiguration('pedal_serial1')},
                {'serial2': LaunchConfiguration('pedal_serial2')},
                {'wait_time_ms': LaunchConfiguration('wait_time_ms')}#kvaserのシリアル番号及びタイムアウト設定
            ] + [params]
        ),
        Node(#受信したデータをmessage id毎にトピック化するノード
            package='wada_vmc',
            namespace='pedal',
            executable='receive_to_topic_20221111',
            parameters=[{'select_can_device': 'pedal'}] + [params],
        ),
        Node(#操舵数値をcanに送信するノード
            package='wada_vmc',
            namespace='pedal',
            executable='kvaser_sender_20221111',
            parameters=[
                {'serial1': LaunchConfiguration('pedal_serial1')},
                {'serial2': LaunchConfiguration('pedal_serial2')},
                {'select_can_device': 'pedal'},
            ],
            remappings=[
                ("can100", "/vehicle/wada_vmc/can100"),
            ],
        ),

        #ステア関連
        Node(#joyboardからデータを受信するノード
            package='wada_vmc',
            namespace='steer',
            executable='kvaser_receiver',
            parameters=[
                {'serial1': LaunchConfiguration('steer_serial1')},
                {'serial2': LaunchConfiguration('steer_serial2')},
                {'wait_time_ms': LaunchConfiguration('wait_time_ms')},#kvaserのシリアル番号及びタイムアウト設定
            ],
        ),
        Node(#受信したデータをmessage id毎にトピック化するノード
            package='wada_vmc',
            namespace='steer',
            executable='receive_to_topic_20221111',
            parameters=[{'select_can_device': 'steer'}] + [params],
            remappings=[
                ("steering_status", "/vehicle/status/steering_status"),
                ("beta", "/localization/pose_twist_fusion_filter/beta"),
            ]
        ),
        Node(#操舵数値をcanに送信するノード
            package='wada_vmc',
            namespace='steer',
            executable='kvaser_sender_20221111',
            parameters=[
                {'serial1': LaunchConfiguration('steer_serial1')},
                {'serial2': LaunchConfiguration('steer_serial2')},
                {'select_can_device': 'steer'},
            ],
            remappings=[
                ("can100", "/vehicle/wada_vmc/can100"),
            ],
        ),

        #自動運転計算
        Node(#ステア自動運転ノード
            package='car_auto_operation',
            namespace='steer',
            executable='car_auto_operation_steer',
            parameters=[params],
            remappings=[
                ("autoware_steer_cmd", "/control/command/control_cmd"),
            ],
        ),
        Node(#autowareの情報からcanの操舵数値を計算するノード
            package='wada_vmc',
            namespace='',
            executable='vmc_calculator_20221111',
            parameters=[params],
            remappings=[
                ("can501", "pedal/can501"),
                ("can502", "steer/can502"),
                ("steer_cmd", "steer/steer_cmd"),
                ("pedal_clutch_latch", "/vehicle/contec/pedal_clutch_latch"),
                ("auto_permission", "/vehicle/auto_permission/auto_permission"),
            ],
        ),
    ]
    
    return nodes

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('pedal_serial1', '101138')#ペダルkvaserのシリアル番号１番目
    add_launch_arg('pedal_serial2', '0')#ペダルkvaserのシリアル番号２番め
    add_launch_arg('steer_serial1', '108346')#ステアkvaserのシリアル番号１番目
    add_launch_arg('steer_serial2', '0')#ステアkvaserのシリアル番号２番め
    add_launch_arg('wait_time_ms', '100')#kvaser受信のウェイトタイム(ミリ秒)

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
