# ros2-farm-scout-FYP

A dual-mode agricultural field scouting robot built on ROS 2 Jazzy and Gazebo Harmonic. The robot navigates a simulated 22×24 m maize field, switching between keyboard teleoperation and autonomous Nav2 navigation at runtime.

This is a Final Year Project (FYP).

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- TurtleBot3 packages (`ros-jazzy-turtlebot3*`, `ros-jazzy-turtlebot3-gazebo`)
- Nav2 (`ros-jazzy-nav2-bringup`)
- SLAM Toolbox (`ros-jazzy-slam-toolbox`)

```bash
sudo apt install \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-joint-state-publisher
```

---

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the simulation

### Gazebo only (no navigation)

```bash
ros2 launch robot_bringup simulation.launch.py
```

### Full system — SLAM mapping mode

Use this the first time, or whenever you need to remap the field.

```bash
ros2 launch robot_bringup full_system.launch.py use_slam:=true
```

Drive the robot around the full field using the keyboard controls below. Once you have full coverage, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/nav2_config/maps/farm_map
```

### Full system — autonomous navigation (AMCL)

Requires a saved map at `nav2_config/maps/farm_map.yaml`.

```bash
ros2 launch robot_bringup full_system.launch.py use_slam:=false
```

Send a goal from RViz or the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{'pose': {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': 4.0, 'y': 2.0}, 'orientation': {'w': 1.0}}}}"
```

---

## Switching modes at runtime

The robot starts in **manual** mode. Switch with:

```bash
# Hand control to Nav2
ros2 topic pub /mode std_msgs/msg/String "data: 'auto'"

# Return to keyboard
ros2 topic pub /mode std_msgs/msg/String "data: 'manual'"
```

If Nav2 stops publishing for more than 0.5 s in auto mode, the robot stops automatically.

---

## Keyboard controls

Run in a real terminal (not an IDE output pane).

| Key | Action |
|---|---|
| `w` / `↑` | Forward |
| `s` / `↓` | Backward |
| `a` / `←` | Turn left |
| `d` / `→` | Turn right |
| `q` / `e` | Forward diagonal |
| `z` / `c` | Backward diagonal |
| `Space` / `x` | Stop |
| `k` | Quit |

Max speed: 0.30 m/s linear.

---

## World layout

The farm world (`farm_world/worlds/farm.sdf`) is a 22×24 m enclosed field.

- **Boundary walls** at x = ±11 m, y = ±12 m, 0.5 m tall
- **5 crop rows** at x = −8, −4, 0, 4, 8 m
- **10 plants per row**, y = −9 to +9 m at 2 m spacing — 50 plants total
- Plants alternate between `maize_01` and `maize_02` photogrammetry meshes
- **Soil patch**: 20×22 m centred at origin (brown)
- **Grass border**: 40×40 m (green), navigation space between soil and walls
- **Obstacles**: rock at (−6, −4), crate at (6, 3), post at (0, −6)
- **Robot spawn**: (0, −11, 0.2) — south of the field, 3 m from the last plant row

Maize meshes come from the [`virtual_maize_field`](https://github.com/FieldRobotEvent/virtual_maize_field) package.

---

## Packages

| Package | Purpose |
|---|---|
| `farm_world` | SDF world file and Gazebo launch |
| `robot_bringup` | Top-level launch files (`simulation`, `navigation`, `full_system`) |
| `nav2_config` | Nav2 params (AMCL + DWB controller) and saved map |
| `mode_arbitration` | Arbitration node — routes `/cmd_vel_auto` or `/cmd_vel_teleop` to `/cmd_vel` |
| `teleop_node` | Keyboard teleoperation node |
| `robot_description` | TurtleBot3 Waffle URDF xacro |

---

## Key topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `TwistStamped` | → robot |
| `/cmd_vel_teleop` | `Twist` | teleop → arbitration |
| `/cmd_vel_auto` | `Twist` | Nav2 → arbitration |
| `/mode` | `String` | external → arbitration |
| `/scan` | `LaserScan` | Gazebo → ROS |
| `/odom` | `Odometry` | Gazebo → ROS |
| `/imu` | `Imu` | Gazebo → ROS |
| `/camera/image_raw` | `Image` | Gazebo → ROS |
| `/map` | `OccupancyGrid` | SLAM/map_server → Nav2 |
