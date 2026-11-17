from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gui = Node(
        package="piper_gui",
        executable="gui_node",
        output="screen",
    )

    return LaunchDescription([gui])