#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    ###### ROBOT DESCRIPTION ######
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("fusion2urdf"),
                    "urdf",
                    "fusion2urdf.xacro",
                ]
            ),

        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    ###### RVIZ ######
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("fusion2urdf"),
            "launch",
            "urdf.rviz",
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ###### GAZEBO ######
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ]),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'fusion2urdf'],
                        output='both')

    # Add the install path model path
    gazebo_env = AppendEnvironmentVariable("GAZEBO_MODEL_PATH", PathJoinSubstitution([get_package_prefix("fusion2urdf"), "share"]))

    return LaunchDescription([
        gazebo_env,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo,
        spawn_entity
    ])