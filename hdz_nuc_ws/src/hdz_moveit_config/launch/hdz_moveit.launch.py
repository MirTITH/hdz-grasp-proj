import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml as ur_load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from hdz_scripts import add_use_sim_time, get_this_package_share_directory, get_this_package_name
from launch.launch_context import LaunchContext
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def common_load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context: LaunchContext, *args, **kwargs):
    this_package_share_directory = get_this_package_share_directory(context)
    this_package_name = get_this_package_name(context)

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    launch_servo = LaunchConfiguration("launch_servo")

    # MoveIt Configuration
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([this_package_share_directory, "srdf", "hdz.srdf.xacro"]),
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_file = os.path.join(this_package_share_directory, "config", "kinematics.yaml")
    robot_description_kinematics = common_load_yaml(kinematics_file)

    robot_description_planning = {
        "robot_description_planning": ur_load_yaml(
            this_package_name,
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = ur_load_yaml(this_package_name, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = ur_load_yaml(this_package_name, "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    moveit_sensor_config = common_load_yaml(os.path.join(this_package_share_directory, "config/sensors_3d.yaml"))

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            # robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": True},
            {"robot_description_kinematics": robot_description_kinematics},
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
            # moveit_sensor_config,
            # {"octomap_frame": "world"},
            # {"octomap_resolution": 0.02},
            # {"max_range": 2.0},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "view_robot.rviz"])
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(gui),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_semantic,
            ompl_planning_pipeline_config,
            {"robot_description_kinematics": robot_description_kinematics},
            robot_description_planning,
            warehouse_ros_config,
        ],
    )

    # Servo node for realtime control
    servo_yaml = ur_load_yaml(this_package_name, "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description_semantic,
        ],
        output="screen",
    )

    nodes_to_start = [move_group_node, rviz_node, servo_node]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )

    add_use_sim_time(declared_arguments, default_value="false")

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("gui", default_value="true", description="Launch RViz?"))
    declared_arguments.append(DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
