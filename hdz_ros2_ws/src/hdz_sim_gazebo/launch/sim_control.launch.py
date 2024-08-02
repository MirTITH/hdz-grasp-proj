import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from hdz_scripts import get_this_package_share_directory, add_use_sim_time
from launch.launch_context import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from hdz_full_description import get_robot_description_content


# helper function to spawn controllers
def controller_spawner(name, *args):
    return Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[name] + [a for a in args],
    )


def launch_setup(context: LaunchContext, *args, **kwargs):
    launch_entities = []
    actions_after_robot_spawn = []  # Actions to be executed after spawn the robot

    use_sim_time = LaunchConfiguration("use_sim_time")
    this_package_share_directory = get_this_package_share_directory(context)
    start_joint_controller = LaunchConfiguration("start_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)

    # Gazebo
    gazebo_world = ""
    # gazebo_world = os.path.join(this_package_share_directory, "gazebo_world/s4.xml")

    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
            launch_arguments={"world": gazebo_world}.items(),
        )
    )

    # Robot State Publisher
    robot_description_content = get_robot_description_content(
        sim_gazebo="true",
        simulation_controllers=os.path.join(this_package_share_directory, "config", "hdz_sim_controllers.yaml"),
    )
    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": robot_description_content,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    # Spawn robot
    robot_spawn_action = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "hdz_full", "-topic", "robot_description"],
        output="screen",
    )
    launch_entities.append(
        TimerAction(
            period=4.0,  # Wait for Gazebo to start
            actions=[robot_spawn_action],
        )
    )

    # Spawn controllers
    # Active controllers
    active_list = [
        "joint_state_broadcaster",
        "gripper_controller",
    ]

    stopped_list = []

    if start_joint_controller == "true":
        active_list.append(initial_joint_controller)
    else:
        stopped_list.append(initial_joint_controller)

    for controller in active_list:
        actions_after_robot_spawn.append(controller_spawner(controller))

    for controller in stopped_list:
        actions_after_robot_spawn.append(controller_spawner(controller, "--stopped"))

    # # rviz
    # rviz_config_file = os.path.join(this_package_share_directory, "rviz", "view_robot_perception.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     # output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[{"use_sim_time": True}],
    # )
    # actions_after_robot_spawn.append(rviz_node)

    launch_entities.append(
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=robot_spawn_action,
                on_completion=actions_after_robot_spawn,
            )
        )
    ),

    return launch_entities


def generate_launch_description():
    declared_arguments = []

    add_use_sim_time(declared_arguments, default_value="false")

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
