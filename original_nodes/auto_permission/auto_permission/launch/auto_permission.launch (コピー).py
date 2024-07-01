import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('gnss_dev', '/sensing/gnss/novatel_oem7/pos_var')#GNSSの位置誤差のトピック
    add_launch_arg('curr_pos', '/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias')#現在の車両位置のトピック
    add_launch_arg('curr_vel', '/localization/pose_twist_fusion_filter/twist_with_covariance')#現在の車両速度のトピック

    container = ComposableNodeContainer(
        name='auto_permission',
        namespace='auto_permission',
        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            ComposableNode(#kvaserを通じて車両canからデータを受信するノード
                package='auto_permission',
                namespace='auto_permission',
                plugin='auto_permission::AutoPermission',#ソースコード内で作成したノードクラス名を指定
                name='auto_permission',
                extra_arguments=[{"use_intra_process_comms": True}],#コンポーネント内でのトピック通信でのメモリコピーを回避する
                #parameters=[
                #    {'gnss_dev': LaunchConfiguration('gnss_dev')},
                #    {'curr_pos': LaunchConfiguration('curr_pos')},
                #    {'curr_vel': LaunchConfiguration('curr_vel')},
                #],
                remappings=[
                    ('gnss_dev', 'gnss_dev'),
                    ('curr_pos', 'curr_pos'),
                    ('curr_vel', 'curr_vel'),
                ],
            ),
        ],

    )

    return launch.LaunchDescription(launch_arguments + [container])
