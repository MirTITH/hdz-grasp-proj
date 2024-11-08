from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

kThisPackageName = "hdz_base_description"


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("urdf_launch"), "launch", "display.launch.py"]),
            launch_arguments={
                "urdf_package": kThisPackageName,
                "urdf_package_path": PathJoinSubstitution(["urdf", "hdz_base.xacro"]),
                "rviz_config": PathJoinSubstitution([FindPackageShare(kThisPackageName), "rviz", "view_robot.rviz"]),
            }.items(),
        )
    )

    return ld
