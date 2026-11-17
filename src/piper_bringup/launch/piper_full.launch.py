import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, filename):
    package_share = get_package_share_directory(package_name)
    launch_file = os.path.join(
        package_share,
        "launch",
        filename,
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file)
    )


def generate_launch_description():
    gazebo = include_launch(
        "piper_bringup",
        "piper_gazebo.launch.py",
    )

    backend = include_launch(
        "piper_control",
        "control.launch.py",
    )

    gui = include_launch(
        "piper_gui",
        "gui.launch.py",
    )

    return LaunchDescription([
        gazebo,
        backend,
        gui,
    ])