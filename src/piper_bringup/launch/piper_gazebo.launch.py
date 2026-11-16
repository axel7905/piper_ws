import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('piper_description')
    gazebo_share = get_package_share_directory('piper_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        description_share, 'urdf', 'piper_description_gazebo.xacro'
    )
    world_file = os.path.join(gazebo_share, 'worlds', 'piper_empty.sdf')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_gazebo_classic': 'false'},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_piper',
        output='screen',
        arguments=[
            '--world', 'piper_world',
            '--topic', '/robot_description',
            '--name', 'piper',
            '--z', '0.0',
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    gripper8_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper8_controller_spawner',
        arguments=[
            'gripper8_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    start_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )
    start_arm_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])
    )
    start_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller],
        )
    )

    start_gripper8_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller,
            on_exit=[gripper8_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        clock_bridge,
        TimerAction(period=5.0, actions=[spawn_robot]),
        start_joint_states,
        start_arm_controller,
        start_gripper_controller,
        start_gripper8_controller,
    ])
