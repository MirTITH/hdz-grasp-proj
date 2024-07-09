from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction

kThisPackageName = "hdz_front_end"


def launch_setup(context, *args, **kwargs):
    launch_entities = []
    launch_entities.append(
        Node(
            package=kThisPackageName,
            executable="hdz_front_end",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    # "robot_description": robot_description_content,
                }
            ],
        )
    )
    return launch_entities


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(declared_arguments)
