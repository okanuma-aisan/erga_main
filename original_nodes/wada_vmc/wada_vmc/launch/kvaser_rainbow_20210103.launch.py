import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('serial1', '77264')#kvaserのシリアル番号１番目
    add_launch_arg('serial2', '0')#kvaserのシリアル番号２番め
    add_launch_arg('wait_time_ms', '10')#kvaser受信のウェイトタイム(秒)
    add_launch_arg('no_device', 'false')#kvaserデバイスにconnectせずにノードを起動したい場合はtrue  将来的には消したい
    add_launch_arg('steer_input_left_min', '-15000')#joyボードのステア入力左回りの最小値(can値)
    add_launch_arg('steer_input_right_max', '15000')#joyボードのステア入力右回りの最大値(can値)
    add_launch_arg('steer_input_voltage_min', '0.0')#joyボードのステア入力電圧最小値
    add_launch_arg('steer_input_voltage_max', '5.0')#joyボードのステア入力電圧最大値
    add_launch_arg('pedal_input_voltage_min', '0.0')#joyボードのペダル入力電圧最小値
    add_launch_arg('pedal_input_voltage_max', '5.0')#joyボードのペダル入力電圧最大値
    add_launch_arg('wheelrad_to_steering_can_value_left', '25009.6727514125')#ホイール角度をハンドル角度に変換する係数(左回り)
    add_launch_arg('wheelrad_to_steering_can_value_right', '26765.9140133745')#ホイール角度をハンドル角度に変換する係数(右回り)

    container = ComposableNodeContainer(
            name='containe_wada_vmc_kvaser',
            namespace='wada_vmc',
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
                    plugin='wada_vmc::ReceiveToTopic20210103',#ソースコード内で作成したノードクラス名を指定
                    name='receive_to_topic',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    remappings=[
                        ("steering_status", "/vehicle/status/steering_status"),
                    ]
                ),
                ComposableNode(#autowareの情報からcanの操舵数値を計算するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::VmcCalculator20210103',#ソースコード内で作成したノードクラス名を指定
                    name='vmc_calculator',
                    extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                    parameters=[
                        {'no_device': LaunchConfiguration('no_device')},
                        {'steer_input_left_min': LaunchConfiguration('steer_input_left_min')},
                        {'steer_input_right_max': LaunchConfiguration('steer_input_right_max')},
                        {'steer_input_voltage_min': LaunchConfiguration('steer_input_voltage_min')},
                        {'steer_input_voltage_max': LaunchConfiguration('steer_input_voltage_max')},
                        {'pedal_input_voltage_min': LaunchConfiguration('pedal_input_voltage_min')},
                        {'pedal_input_voltage_max': LaunchConfiguration('pedal_input_voltage_max')},
                        {'wheelrad_to_steering_can_value_left': LaunchConfiguration('wheelrad_to_steering_can_value_left')},
                        {'wheelrad_to_steering_can_value_right': LaunchConfiguration('wheelrad_to_steering_can_value_right')},
                    ]
                ),
                ComposableNode(#操舵数値をcanに送信するノード
                    package='wada_vmc',
                    namespace='wada_vmc',
                    plugin='wada_vmc::KvaserSender20210103',#ソースコード内で作成したノードクラス名を指定
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
