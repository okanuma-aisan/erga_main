import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('serial1', '97692')#kvaserのシリアル番号１番目
    add_launch_arg('serial2', '0')#kvaserのシリアル番号２番め
    add_launch_arg('wait_time_ms', '10')#kvaser受信のウェイトタイム(ミリ秒)

    container = ComposableNodeContainer(
        name='containe_wada_vmc_kvaser',
        namespace='',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            #ペダル関連
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
                ],
            ),
        ],
    )

    return launch.LaunchDescription(launch_arguments + [container])
