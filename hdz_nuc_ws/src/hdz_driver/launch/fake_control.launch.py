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
from hdz_scripts import get_this_package_share_directory


# helper function to spawn controllers
def controller_spawner(name, *args):
    return Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[name] + [a for a in args],
    )


def launch_setup(context, *args, **kwargs):
    launch_entities = []

    this_package_share_directory = get_this_package_share_directory(context)

    ag95_com_port = LaunchConfiguration("ag95_com_port").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)
    gui = LaunchConfiguration("gui").perform(context)

    robot_description_content = get_robot_description_content(
        ag95_com_port=ag95_com_port,
        use_fake_hardware=use_fake_hardware,
        fake_sensor_commands=fake_sensor_commands,
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": robot_description_content,
                    "use_sim_time": False,
                }
            ],
        )
    )

    # Load controllers configuration
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [
                this_package_share_directory,
                "config",
                "hdz_controllers.yaml",
            ]
        ),
        allow_substs=True,
    )

    # controller manager node
    launch_entities.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            output="both",
            remappings=[
                ("controller_manager/robot_description", "robot_description"),
            ],
        )
    )

    # rviz node
    if gui == "true":
        rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "hdz_control.rviz"])
        launch_entities.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
            )
        )

    # Active controllers
    active_list = [
        "joint_state_broadcaster",
        # "gripper_forward_position_controller",
        "gripper_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "joint_trajectory_controller",
    ]

    for controller in active_list:
        launch_entities.append(controller_spawner(controller))

    # Inactive controllers
    inactive_list = ["forward_position_controller"]

    for controller in inactive_list:
        launch_entities.append(controller_spawner(controller, "--inactive"))

    return launch_entities


def generate_launch_description():
    launch_entities = []

    launch_entities.append(
        DeclareLaunchArgument(
            "ag95_com_port",
            default_value="",
            description="COM port of the gripper.",
        )
    )

    launch_entities.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    launch_entities.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable mocked command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )

    launch_entities.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Do not set this for now
    launch_entities.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Do not set this for now! tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    launch_entities.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_entities)
