"""
Full system launch — bring up the complete agricultural field scouting robot.

Starts:
  1. Gazebo Harmonic farm world
  2. robot_state_publisher + ROS↔Gz bridge
  3. Nav2 stack (SLAM or saved-map + static-transform mode)
  4. Mode-arbitration node  (routes /cmd_vel_auto or /cmd_vel_teleop → /cmd_vel)
  5. Keyboard teleoperation node

Arguments:
  use_sim_time  (default true)
  use_slam      (default false)   — true=SLAM, false=saved map + static map->odom transform
  map_file      (default ~/ros2_ws/src/nav2_config/maps/farm_map.yaml) — path to map YAML used with the static-transform localisation
  nav2          (default true)    — false=skip the full Nav2 stack (mapping-only
                                     runs, e.g. slam_coverage_drive.py, which never
                                     touches Nav2/AMCL — only slam_toolbox comes up)

Quick-start:
  # Terminal 1 — bring everything up (SLAM mode for first run)
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_bringup full_system.launch.py use_slam:=true

  # Terminal 1, mapping-only variant — skip the unused Nav2 stack while driving
  # slam_coverage_drive.py (saves significant CPU/RAM; nothing else changes)
  ros2 launch robot_bringup full_system.launch.py use_slam:=true nav2:=false

  # Terminal 2 — switch to autonomous navigation
  ros2 topic pub /mode std_msgs/msg/String "data: 'auto'"

  # Terminal 3 — send a Nav2 goal via RViz or CLI
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{'pose': {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': 4.0, 'y': 2.0}, 'orientation': {'w': 1.0}}}}"

  # Switch back to manual
  ros2 topic pub /mode std_msgs/msg/String "data: 'manual'"

Prerequisites (install once):
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \\
                   ros-jazzy-joint-state-publisher
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam     = LaunchConfiguration('use_slam',     default='false')
    map_file     = LaunchConfiguration('map_file')
    nav2         = LaunchConfiguration('nav2',         default='true')

    bringup_share = FindPackageShare('robot_bringup')

    # ── 1. Simulation (Gazebo + robot + bridge) ───────────────────────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. Navigation stack (delayed to let Gazebo start publishing /scan /odom
    #      and to let the /tf stream stabilize before Nav2/SLAM TF listeners attach)
    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, 'launch', 'navigation.launch.py']
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_slam':     use_slam,
                    'map_file':     map_file,
                    'nav2':         nav2,
                }.items(),
            )
        ],
    )

    # ── 3. Mode-arbitration node ──────────────────────────────────────────────
    arbitration = Node(
        package='mode_arbitration',
        executable='arbitration_node',
        name='arbitration_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_slam',     default_value='false',
                              description='true=SLAM Toolbox, false=static transform localization + map'),
        DeclareLaunchArgument('map_file',
                              default_value='/home/ssrlserg1/ros2_ws/src/nav2_config/maps/farm_map.yaml',
                              description='Absolute path to map YAML (static transform localization mode)'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='false = skip the full Nav2 stack, slam_toolbox only'),
        simulation,
        arbitration,
        navigation,
    ])
