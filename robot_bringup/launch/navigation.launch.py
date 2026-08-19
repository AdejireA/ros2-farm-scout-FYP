"""
Navigation launch — Nav2 stack with AMCL or SLAM Toolbox.

Arguments:
  use_slam   (default false) — true = online SLAM, false = AMCL + saved map
  map_file   (default '')    — absolute path to map YAML (only used when use_slam=false)
  use_sim_time (default true)
  nav2       (default true)  — false = skip the full Nav2 stack (controller/planner/
                                bt_navigator/behavior/waypoint_follower/smoother/AMCL)
                                entirely; only slam_toolbox (if use_slam=true) is brought
                                up, under its own minimal lifecycle manager. For mapping-
                                only runs (e.g. slam_coverage_drive.py, which drives via
                                direct /cmd_vel_teleop and never touches Nav2) — saves the
                                CPU/RAM the unused Nav2 stack would otherwise cost.

Nav2 controller output is remapped:
  /cmd_vel          →  /cmd_vel_auto
  /cmd_vel_smoothed →  /cmd_vel_auto
so that the mode_arbitration node controls final robot velocity.

Prerequisites:
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_slam_str      = LaunchConfiguration('use_slam').perform(context)
    map_file          = LaunchConfiguration('map_file').perform(context)
    nav2_str          = LaunchConfiguration('nav2').perform(context)

    use_sim_time = use_sim_time_str.lower() in ('true', '1')
    use_slam     = use_slam_str.lower() in ('true', '1')
    nav2_enabled = nav2_str.lower() in ('true', '1')

    params_file = PathJoinSubstitution(
        [FindPackageShare('nav2_config'), 'config', 'nav2_params.yaml']
    )

    common = [params_file, {'use_sim_time': use_sim_time}]

    if not nav2_enabled:
        # Mapping-only path: skip the full Nav2 stack entirely (not used by
        # slam_coverage_drive.py, which drives via direct /cmd_vel_teleop and
        # never touches Nav2/AMCL) — only slam_toolbox goes up, if requested,
        # under its own minimal lifecycle manager so it still reaches ACTIVE.
        light_nodes = []
        if use_slam:
            light_nodes.append(
                LifecycleNode(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    namespace='',
                    output='screen',
                    parameters=common,
                )
            )
            light_nodes.append(
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam_only',
                    namespace='',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['slam_toolbox']},
                    ],
                )
            )
        return light_nodes

    # ── Shared Nav2 nodes (always active) ────────────────────────────────────
    nav2_nodes = []

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            output='screen',
            parameters=common,
            # Remap controller output so arbitration node controls /cmd_vel
            remappings=[('cmd_vel', 'cmd_vel_auto')],
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace='',
            output='screen',
            parameters=common,
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            output='screen',
            parameters=common,
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace='',
            output='screen',
            parameters=common,
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            output='screen',
            parameters=common,
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
            output='screen',
            parameters=common,
        )
    )

    nav2_nodes.append(
        LifecycleNode(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace='',
            output='screen',
            parameters=common,
            # Remap smoother output too (it is the final velocity command)
            remappings=[('cmd_vel_smoothed', 'cmd_vel_auto')],
        )
    )

    # ── Localisation: AMCL + map_server OR slam_toolbox ──────────────────────
    if use_slam:
        nav2_nodes.append(
            LifecycleNode(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace='',
                output='screen',
                parameters=common,
            )
        )
        managed = [
            'slam_toolbox',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]
        lm_name = 'lifecycle_manager_slam'
    else:
        map_params = [params_file, {'use_sim_time': use_sim_time}]
        if map_file:
            map_params.append({'yaml_filename': map_file})

        nav2_nodes.append(
            LifecycleNode(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace='',
                output='screen',
                parameters=map_params,
            )
        )
        nav2_nodes.append(
            LifecycleNode(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='',
                output='screen',
                parameters=common,
            )
        )
        managed = [
            'map_server',
            'amcl',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]
        lm_name = 'lifecycle_manager'

    # ── Lifecycle manager ─────────────────────────────────────────────────────
    nav2_nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=lm_name,
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'bond_timeout': 0.0},
                {'node_names': managed},
            ],
        )
    )

    return nav2_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from simulation'),
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='true=SLAM Toolbox, false=AMCL+map'),
        DeclareLaunchArgument('map_file',
                              default_value='/home/adejirea/ros2_ws/src/nav2_config/maps/farm_map.yaml',
                              description='Absolute path to map YAML for AMCL mode'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='false = skip the full Nav2 stack, slam_toolbox only'),
        OpaqueFunction(function=launch_setup),
    ])
