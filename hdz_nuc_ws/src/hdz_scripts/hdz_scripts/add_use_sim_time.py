from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import inspect


def add_use_sim_time(launch_entities: list, default_value: str = "false"):
    launch_entities.append(
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use simulation (Gazebo) clock if true",
            choices=["true", "false"],
            default_value=default_value,
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 获取调用者的堆栈帧
    caller_frame = inspect.stack()[1]
    # 获取调用者所在文件的路径
    caller_filepath = caller_frame.filename
    launch_entities.append(
        ExecuteProcess(
            cmd=["echo", f"use_sim_time:", use_sim_time],
            output="both",
            name=caller_filepath,
        )
    )

    return use_sim_time
