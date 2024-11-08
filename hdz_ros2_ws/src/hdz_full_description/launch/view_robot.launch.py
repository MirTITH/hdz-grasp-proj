from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Tuple, Dict, List
from hdz_scripts import add_use_sim_time, get_this_package_share_directory
from hdz_full_description import get_robot_description_content


def launch_setup(context, *args, **kwargs):
    launch_entities = []

    this_package_share_directory = get_this_package_share_directory(context)

    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": get_robot_description_content(),
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    # Already hard-coded in d435i.xacro file, so no need to launch it here

    # launch_entities.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory("easy_handeye2"), "launch", "publish.launch.py")
    #         ),
    #         launch_arguments={
    #             "name": "hdz_handeye2",
    #         }.items(),
    #     )
    # )

    launch_entities.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "view_robot.rviz"])

    launch_entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return launch_entities


def generate_launch_description():
    launch_entities = []

    add_use_sim_time(launch_entities, default_value="false")

    launch_entities.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_entities)
