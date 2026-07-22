"""Launch only the Gazebo Harmonic farm world (no robot)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = PathJoinSubstitution(
        [FindPackageShare('farm_world'), 'worlds', 'farm.sdf']
    )

    gz_args = LaunchConfiguration('gz_args', default=['-r ', world_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r ', world_file],
            description='Arguments passed to gz sim',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                )
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ])
