# ros2-farm-scout-FYP

A dual-mode agricultural field scouting robot, built as a Final Year Project using ROS 2 Jazzy and Gazebo Harmonic.

The robot's job is **route traversal**: it drives down each aisle of a 5-row maize field in sequence — a boustrophedon (lawnmower) pattern — visiting every row from end to end before returning to its start position. This is the scouting mission profile. The FYP investigates how the mode-switching architecture holds up during that mission under different failure conditions.

Two modes are available at runtime:

- **Manual** — a keyboard teleop node drives the robot. This is the default on startup.
- **Auto** — Nav2 takes over, executing the waypoint sequence via `nav2_waypoint_follower`.

Switching is done by publishing to `/mode`. If Nav2 goes silent for more than 0.5 seconds while in auto mode, the robot stops. That fallback is the core safety property this project tests.

---

## Scouting mission profile

The mission sends the robot down five aisles at x = −8, −4, 0, 4, 8 m, alternating direction each time (south→north, north→south, ...) so it never crosses back through terrain it just covered. After the last aisle it returns to spawn.

```
start (0, -11)
  │
  ▼
x=-8  south→north   x=-4  north→south   x=0  south→north
                                                    │
x=4   north→south   x=8   south→north ◄────────────┘
  │
  ▼
return to (0, -11)
```

11 waypoints total. Run it with:

```bash
ros2 run robot_bringup scout_mission
```

Nav2 must be active and the robot localised before calling this.

---

## Robot specs

The robot modelled in this project is a custom differential-drive platform.

| Property | Value |
|---|---|
| Dimensions | 0.60 × 0.40 × 0.20 m |
| Mass | 25 kg |
| Drive | Differential (2 wheels + passive caster) |
| Wheel separation | 0.46 m, radius 0.15 m |
| Max speed | 0.35 m/s linear, 0.80 rad/s angular |
| LIDAR | 360°, 12 m range, 10 Hz, Gaussian noise σ=0.01 |
| Camera | RGB 640×480, 30 Hz |
| IMU | 100 Hz |

The Gazebo simulation uses a TurtleBot3 Waffle as a stand-in for physics and sensors.

---

## Field environment

The farm world (`farm_world/worlds/farm.sdf`) is a 22×24 m enclosed field with 0.5 m boundary walls at x = ±11 m, y = ±12 m.

- 5 crop rows at x = −8, −4, 0, 4, 8 m
- 10 maize plants per row, y = −9 to +9 m at 2 m spacing (50 plants total)
- Plants alternate between two photogrammetry mesh models from the [`virtual_maize_field`](https://github.com/FieldRobotEvent/virtual_maize_field) package
- 3 static obstacles: rock at (−6, −4), crate at (6, 3), post at (0, −6)
- Robot spawns at (0, −11, 0.2) — south of the planting area, facing north

The grass border between the soil patch and the walls is intentional navigation space, giving the robot room to turn at the ends of rows.

---

## Architecture

```
/mode ──────────────────────────────────────────┐
                                                ▼
keyboard_teleop ──► /cmd_vel_teleop ──┐
                                      ▼
Nav2 ──► /cmd_vel_auto ───────────► arbitration_node  (default: MANUAL)
                                                │
                                                ▼
                                           /cmd_vel
                                                │
                                       Gazebo diff-drive
                                                │
                                    /scan   /odom   /tf
```

The arbitration node is the only thing that writes to `/cmd_vel`. Nav2 and the teleop node write to separate topics and the arbitration node picks one based on the current mode.

---

## Requirements

```bash
sudo apt install \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-joint-state-publisher
```

---

## External dependencies

The maize plant meshes come from the [`virtual_maize_field`](https://github.com/FieldRobotEvent/virtual_maize_field) package. It's **not** part of this repo (gitignored — it's a large third-party asset package, not code we own) and must be cloned alongside it:

```bash
cd ~/ros2_ws/src
git clone https://github.com/FieldRobotEvent/virtual_maize_field.git
```

Without this, Gazebo will fail to load the farm world (`model://maize_01` / `maize_02` not found).

---

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running

### Simulation only

```bash
ros2 launch robot_bringup simulation.launch.py
```

### Full system — SLAM (first run / remapping)

```bash
ros2 launch robot_bringup full_system.launch.py use_slam:=true
```

Drive around the full field, then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/nav2_config/maps/farm_map
```

### Full system — autonomous navigation

Requires a saved map at `nav2_config/maps/farm_map.yaml`.

```bash
ros2 launch robot_bringup full_system.launch.py use_slam:=false
```

Switch to auto mode and send a goal:

```bash
ros2 topic pub /mode std_msgs/msg/String "data: 'auto'"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{'pose': {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': 4.0, 'y': 2.0}, 'orientation': {'w': 1.0}}}}"
```

Return to manual:

```bash
ros2 topic pub /mode std_msgs/msg/String "data: 'manual'"
```

---

## Keyboard controls

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

Run in a real terminal — keyboard capture does not work in IDE output panes.

---

## Packages

| Package | Purpose |
|---|---|
| `farm_world` | SDF world file and Gazebo launch |
| `robot_bringup` | Top-level launch files |
| `nav2_config` | Nav2 params (AMCL + DWB controller) and saved map |
| `mode_arbitration` | Routes `/cmd_vel_auto` or `/cmd_vel_teleop` → `/cmd_vel` |
| `teleop_node` | Keyboard teleoperation |
| `robot_description` | Custom agricultural robot URDF xacro |
