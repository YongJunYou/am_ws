from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Config 파일 경로
    config_file = PathJoinSubstitution([
        FindPackageShare('mani_controller'),
        'config',
        'mani_controller.yaml'
    ])

    # mani_controller 노드
    mani_controller_node = Node(
        package='mani_controller',
        executable='mani_node',
        name='mani_controller',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        mani_controller_node
    ])

