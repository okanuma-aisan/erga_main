import launch
import os
import yaml
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def get_vehicle_info(context):
    path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]

    #p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
    #p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
    #p["min_longitudinal_offset"] = -p["rear_overhang"]
    #p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
    #p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
    #p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
    #p["min_height_offset"] = 0.0
    #p["max_height_offset"] = p["vehicle_height"]
    return p

def get_params(context):
    #path = os.path.join(
    #    get_package_share_directory('detect_fusion'),
    #    'config',
    #    'detect_fusion.param.yaml'
    #)

    path = LaunchConfiguration("param_file").perform(context)

    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params(context)
    vehicle_params = get_vehicle_info(context)

    nodes = [
        Node(
            package='detect_fusion',
            executable='detect_fusion',
            parameters = [params] + [
                {'wheel_base': vehicle_params["wheel_base"]},
                {'front_overhang': vehicle_params["front_overhang"]},
            ],
            remappings=[
                ("in_mobileye_detect", LaunchConfiguration('in_mobileye_detect')),
                ("in_lidar_detect", LaunchConfiguration('in_lidar_detect')),
                ("in_kinematic_state", LaunchConfiguration('in_kinematic_state')),
                ("out_fusion_detect", LaunchConfiguration('out_fusion_detect')),
            ],
        ),
    ]

    return nodes

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("sample_vehicle_description"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    add_launch_arg(
        "param_file",
        [
            FindPackageShare("detect_fusion"),
            "/config/detect_fusion.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    add_launch_arg('in_mobileye_detect', '/sensing/mobileye/obstacle_data')
    add_launch_arg('in_lidar_detect', 'objects_centerpoint')
    add_launch_arg('in_kinematic_state', '/localization/kinematic_state')
    add_launch_arg('out_fusion_detect', 'fusion_objects')

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
