from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Tuple, Dict, List
from hdz_full_description import get_robot_description_content
from launch_ros.parameter_descriptions import ParameterFile
from hdz_scripts import get_this_package_share_directory, add_use_sim_time


def launch_setup(context, *args, **kwargs):
    launch_entities = []

    # Include realsense2_camera rs_launch.py
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
            ),
            launch_arguments={
                "pointcloud.enable": "true",
                "align_depth.enable": "true",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
            }.items(),
        )
    )

    return launch_entities


def generate_launch_description():
    launch_entities = []

    add_use_sim_time(launch_entities)

    launch_entities.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_entities)
