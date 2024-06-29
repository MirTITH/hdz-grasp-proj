import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from hdz_scripts import get_this_package_name, add_use_sim_time
from launch.launch_context import LaunchContext

def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.load(file, Loader=yaml.FullLoader)


def launch_setup(context: LaunchContext, *args, **kwargs):
    this_package_name = get_this_package_name(context)
    launch_entities = []

    use_sim_time = LaunchConfiguration("use_sim_time")

    kinematics_file = os.path.join(
        get_package_share_directory("hdz_moveit_config"), "config", "kinematics.yaml"
    )
    robot_description_kinematics = load_yaml(kinematics_file)

    launch_entities.append(
        Node(
            package=this_package_name,
            executable="hdz_grpc_server",
            # output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_description_kinematics": robot_description_kinematics},
                {"max_velocity_scaling_factor": 0.15},
                {"max_acceleration_scaling_factor": 0.15},
                {"planning_group": "ur_manipulator"},
            ],
        )
    )

    return launch_entities


def generate_launch_description():
    declared_arguments = []
    add_use_sim_time(declared_arguments)

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
