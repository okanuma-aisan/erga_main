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

def get_params():
    path = os.path.join(
        get_package_share_directory('transform_center_velocity'),
        'config',
        'erga.param.yaml'
    )

    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    params = get_params()
    vehicle_params = get_vehicle_info(context)

    nodes = [
        Node(
           package='transform_center_velocity',
           executable='transform_center_velocity',
           parameters = [params] + [
               {'wheel_base': vehicle_params["wheel_base"]},
           ]
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

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
