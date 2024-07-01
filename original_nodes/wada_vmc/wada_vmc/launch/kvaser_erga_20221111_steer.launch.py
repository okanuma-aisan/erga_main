import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('serial1', '108346')#kvaserのシリアル番号１番目
    add_launch_arg('serial2', '0')#kvaserのシリアル番号２番め
    add_launch_arg('select_can_device', 'steer')#担当車両デバイス
    add_launch_arg('wait_time_ms', '10')#kvaser受信のウェイトタイム(ミリ秒)
    add_launch_arg('steer_input_center', '1024')#joyボードのステア中央値
    add_launch_arg('steer_input_left_max', '2048')#joyボードのステア入力左回りの最大値
    add_launch_arg('steer_input_right_min', '-0')#joyボードのステア入力右回りの最小値
    #add_launch_arg('wheelrad_to_steering_can_value_left', '25009.6727514125')#ホイール角度をハンドル角度に変換する係数(左回り)
    #add_launch_arg('wheelrad_to_steering_can_value_right', '26765.9140133745')#ホイール角度をハンドル角度に変換する係数(右回り)
    add_launch_arg('steering_can_value_left_to_wheelrad_slope', '0.000926189')#ホイール角度をハンドル角度に変換する係数(左回り)
    add_launch_arg('steering_can_value_left_to_wheelrad_intercept', '0.050007438')#ホイール角度をハンドル角度に変換する係数(左回り)
    add_launch_arg('steering_can_value_right_to_wheelrad_slope', '0.000887424')#ホイール角度をハンドル角度に変換する係数(右回り)
    add_launch_arg('steering_can_value_right_to_wheelrad_intercept', '0.031315874')#ホイール角度をハンドル角度に変換する係数(右回り)


    container = ComposableNodeContainer(
            name='containe_wada_vmc_kvaser',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            
            composable_node_descriptions=[
                ComposableNode(#kvaserを通じて車両canからデータを受信するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::KvaserReceiver',#ソースコード内で作成したノードクラス名を指定
                    name='kvaser_receiver',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    parameters=[
                        {'serial1': LaunchConfiguration('serial1')},
                        {'serial2': LaunchConfiguration('serial2')},
                        {'wait_time_ms': LaunchConfiguration('wait_time_ms')},#kvaserのシリアル番号及びタイムアウト設定
                    ]
                ),
                ComposableNode(#受信したデータをmessage id毎にトピック化するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::ReceiveToTopic20221111',#ソースコード内で作成したノードクラス名を指定
                    name='receive_to_topic',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    parameters=[
                        {'steering_can_value_left_to_wheelrad_slope': LaunchConfiguration('steering_can_value_left_to_wheelrad_slope')},
                        {'steering_can_value_left_to_wheelrad_intercept': LaunchConfiguration('steering_can_value_left_to_wheelrad_intercept')},
                        {'steering_can_value_right_to_wheelrad_slope': LaunchConfiguration('steering_can_value_right_to_wheelrad_slope')},
                        {'steering_can_value_right_to_wheelrad_intercept': LaunchConfiguration('steering_can_value_right_to_wheelrad_intercept')},
                        {'select_can_device': LaunchConfiguration('select_can_device')},
                    ],
                    remappings=[
                        ("steering_status", "/vehicle/status/steering_status"),
                    ],
                ),
                ComposableNode(#autowareの情報からcanの操舵数値を計算するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::VmcCalculator20221111',#ソースコード内で作成したノードクラス名を指定
                    name='vmc_calculator',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    parameters=[
                        {'steer_input_center': LaunchConfiguration('steer_input_center')},
                        {'steer_input_left_max': LaunchConfiguration('steer_input_left_max')},
                        {'steer_input_right_min': LaunchConfiguration('steer_input_right_min')},
                    ]
                ),
                ComposableNode(#操舵数値をcanに送信するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::KvaserSender20221111',#ソースコード内で作成したノードクラス名を指定
                    name='kvaser_sender',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    parameters=[
                        {'serial1': LaunchConfiguration('serial1')},
                        {'serial2': LaunchConfiguration('serial2')},
                    ]
                ),
            ],
    )

    return launch.LaunchDescription(launch_arguments + [container])
