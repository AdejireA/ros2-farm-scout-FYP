"""Launch robot_state_publisher + joint_state_publisher_gui + rviz2 for URDF inspection."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'agricultural_robot.urdf.xacro'])

    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
