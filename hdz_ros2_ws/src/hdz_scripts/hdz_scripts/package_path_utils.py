import os
from launch.substitutions import ThisLaunchFileDir
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory


def get_this_package_name(context: LaunchContext):
    this_launch_file_dir = ThisLaunchFileDir().perform(context)
    return os.path.basename(os.path.dirname(this_launch_file_dir))


def get_this_package_share_directory(context: LaunchContext):
    return get_package_share_directory(get_this_package_name(context))
