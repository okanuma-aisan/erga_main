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
        'wada_vmc_rainbow.param.yaml'
        )
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()

    container = ComposableNodeContainer(
        name='containe_wada_vmc_kvaser',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            ComposableNode(#kvaserを通じて車両canからデータを受信するノード
                package='wada_vmc',
                namespace='',
                plugin='wada_vmc::KvaserReceiver',#ソースコード内で作成したノードクラス名を指定
                name='kvaser_receiver',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[
                    {'serial1': LaunchConfiguration('serial1')},
                    {'serial2': LaunchConfiguration('serial2')},
                    {'wait_time_ms': LaunchConfiguration('wait_time_ms')},#kvaserのシリアル番号及びタイムアウト設定
                ],
            ),
            ComposableNode(#受信したデータをmessage id毎にトピック化するノード
                package='wada_vmc',
                namespace='',
                plugin='wada_vmc::ReceiveToTopic20210103',#ソースコード内で作成したノードクラス名を指定
                name='receive_to_topic',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[{'select_can_device': 'all'}] + [params],
                #remappings=[
                #    ("steering_status", "/vehicle/status/steering_status"),
                #],
            ),
            ComposableNode(#操舵数値をcanに送信するノード
                package='wada_vmc',
                namespace='pedal',
                plugin='wada_vmc::KvaserSender20210103',#ソースコード内で作成したノードクラス名を指定
                name='kvaser_sender',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[
                    {'serial1': LaunchConfiguration('serial1')},
                    {'serial2': LaunchConfiguration('serial2')},
                ],
                remappings=[
                    ("can100", "/vehicle/can100"),
                ],
            ),

            #自動運転計算
            ComposableNode(#ステア自動運転ノード
                package='car_auto_operation',
                namespace='steer',
                plugin='car_auto_operation::CarAutoOperationSteer20210103',#ソースコード内で作成したノードクラス名を指定
                name='operation_steer',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[params],
                #remappings=[
                #    ("can502", "steer/can502"),
                #],
            ),
            ComposableNode(#autowareの情報からcanの操舵数値を計算するノード
                package='wada_vmc',
                namespace='',
                plugin='wada_vmc::VmcCalculator20210103',#ソースコード内で作成したノードクラス名を指定
                name='vmc_calculator',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[params],
                remappings=[
                    #("can501", "pedal/can501"),
                    #("can502", "steer/can502"),
                    ("steer_cmd", "steer/steer_cmd"),
                ],
            ),
            
            #自動運転許可ノード
            ComposableNode(#自動運転許可ノード
                package='auto_permission',
                namespace='',
                plugin='auto_permission::AutoPermission',#ソースコード内で作成したノードクラス名を指定
                name='auto_permission',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                parameters=[params],
                remappings=[
                    ("gnss_dev", "/sensing/gnss/novatel_oem7/pos_var"),
                    ("nearest_voxel_transformation_likelihood", "/localization/pose_estimator/nearest_voxel_transformation_likelihood"),
                    ("transform_probability", "/localization/pose_estimator/transform_probability"),
                    ("curr_pos", "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias"),
                    ("curr_vel", "/localization/pose_twist_fusion_filter/twist_with_covariance"),
                    ("pedal_info", "can501"),
                    ("steer_info", "can502"),
                ],
            ),
        ],
    )
    
    return[container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('serial1', '0')#ペダルkvaserのシリアル番号１番目
    add_launch_arg('serial2', '0')#ペダルkvaserのシリアル番号２番め
    add_launch_arg('wait_time_ms', '10')#kvaser受信のウェイトタイム(ミリ秒)

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
