import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('mani_urdf'),
        'rviz',
        'view_mani.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        on_exit=Shutdown()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('mani_urdf'),
                        'urdf',
                        'mani.urdf.xacro',
                    ]),
                ]),
        }]
    )

    urdf_broadcaster_node = Node(
        package='mani_urdf',
        executable='urdf_broadcaster',
        name='urdf_broadcaster',
        output='screen'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='use joint_state_publisher_gui'
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    ld.add_action(use_gui_arg)
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(urdf_broadcaster_node)
    ld.add_action(jsp_gui_node)

    return ld
