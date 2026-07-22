"""
Simulation launch — Gazebo Harmonic farm world + TurtleBot3 Waffle spawn + ROS↔Gz bridge.

Topics after launch:
  /scan               — LaserScan from lidar
  /odom               — Odometry from diff-drive plugin
  /imu                — Imu from IMU sensor
  /camera/image_raw   — Image from front camera
  /tf                 — Transform tree (odom→base_footprint)
  /cmd_vel            — TwistStamped input to robot (written by mode_arbitration)
  /joint_states       — Wheel joint states
"""

import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TurtleBot3 Waffle URDF for robot_state_publisher (Nav2 needs the TF tree)
    tb3_urdf = '/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf'

    # robot_state_publisher reads the URDF (processed by xacro for namespace arg)
    robot_description = Command(['xacro ', tb3_urdf])

    world_file = PathJoinSubstitution(
        [FindPackageShare('farm_world'), 'worlds', 'farm.sdf']
    )

    # ── Gazebo Harmonic ──────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    # ── robot_state_publisher ────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time}
        ],
    )

    # ── ROS ↔ Gazebo topic bridge ────────────────────────────────────────────
    #   Notation:  @  bidirectional
    #              ]  ROS publishes  → Gz subscribes
    #              [  Gz publishes   → ROS subscribes
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # TwistStamped→gz.msgs.Twist: bridge strips header; diff-drive accepts gz.msgs.Twist
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            # Odometry: Gz → ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Lidar scan: Gz → ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera image + info: Gz → ROS
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # IMU: Gz → ROS
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Transform tree from diff-drive: Gz → ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Wheel joint states: Gz → ROS (required by robot_state_publisher)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Simulation clock: Gz → ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/opt/ros/jazzy/share/turtlebot3_gazebo/models',
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.expanduser('~/ros2_ws/src/virtual_maize_field/models'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_sim,
        rsp,
        bridge,
    ])
