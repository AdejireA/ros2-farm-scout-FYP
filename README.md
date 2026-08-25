# ros2-farm-scout-FYP

A dual-mode agricultural field scouting robot built as a Final Year Project using ROS 2 Jazzy and Gazebo Harmonic.

The robot navigates autonomously through the field using Nav2, while an operator can take manual control at any time through a mode-switching arbitration mechanism. The arbitration node is the core contribution of this project — it sits between Nav2 and the teleop interface, ensuring only one velocity source controls the robot at any time. If Nav2 goes silent for more than 0.5 seconds while in AUTO mode, the robot stops automatically.

## System Architecture

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

The arbitration node is the only publisher to `/cmd_vel`. Nav2 and teleop write to separate topics; the arbitration node selects one based on the current mode.

## Field Environment

The farm world (`farm_world/worlds/farm.sdf`) is a 22×24 m enclosed field:

- **Walls:** x = ±11 m, y = ±12 m (0.5 m high boundary walls)
- **Crop rows:** 5 rows at x = -8, -4, 0, 4, 8 m (10 maize plants per row, 50 total)
- **Aisles:** Navigable gaps centered at x = -6, -2, 2, 6 m (2 m from each flanking row)
- **Obstacles:** Rock at (-6, -4), crate at (6, 3), post at (0, -6)
- **Robot spawn:** (-9.5, -10.0), facing north

Plant meshes come from the [virtual_maize_field](https://github.com/FieldRobotEvent/virtual_maize_field) package (not included — must be cloned separately).

## Robot

The simulation uses a TurtleBot3 Waffle as the robot platform:

| Property | Value |
|---|---|
| LiDAR range | 3.5 m, 360°, 10 Hz |
| Max linear speed | 0.35 m/s (Nav2 configured) |
| Max angular speed | 0.80 rad/s |
| Camera | RGB 1920×1080, 30 Hz |
| IMU | 200 Hz |

A separate, custom agricultural-robot URDF exists in `robot_description/` (0.60×0.40×0.20 m, 25 kg, 12 m LiDAR) but is **not** used by any simulation or navigation launch file — only its own standalone `display.launch.py` RViz-inspection tool loads it. Every number above describes the TurtleBot3 Waffle actually spawned by `farm_world/worlds/farm.sdf` and loaded by `robot_state_publisher` in `simulation.launch.py`.

## Scouting Mission

`robot_bringup/scripts/scout_mission.py` is a short, scripted test route — not a full field-coverage sweep. It sends the robot through 6 waypoints across two aisles (x = -6 and x = -2), using shorter segments specifically to avoid Nav2/localisation drift over long single-goal distances, and pauses twice (after waypoint 2 and waypoint 4) to run a scripted "operator takeover" test: switch to MANUAL, hold for 3 s, switch back to AUTO. Each waypoint is sent individually via `goToPose()` rather than a single `followWaypoints()` call.

The mission requires `bt_navigator` and `map_server` to be active, so it must be run against the default (`use_slam:=false nav2:=true`) launch configuration below — it will not work against the `use_slam:=true nav2:=false` SLAM-only path, which never brings up `map_server`.

```bash
python3 robot_bringup/scripts/scout_mission.py --auto --log trial_01
```

With `--log <name>`, per-waypoint results (status, duration, mode switches) are written to `~/ros2_ws/<name>.csv`. Omit `--auto` to pause for Enter between waypoints instead of running straight through.

## Map & Localisation

There is no code in this repository that generates a map from the world's known SDF geometry. The map used for navigation (`farm_map.yaml` / `farm_map.pgm`) is an ordinary saved SLAM Toolbox map: drive the robot in `use_slam:=true` mode, then save it with `nav2_map_server`'s `map_saver_cli`.

Rather than re-localising against that map with AMCL, `navigation.launch.py` publishes a single **static** `map → odom` transform (via `tf2_ros static_transform_publisher`) instead. This works because the SLAM session that produced the map was started with the robot at the known spawn pose — the static transform is fixed to that exact pose (map (-9.5, -10), yaw 90°, corresponding to odom's origin at (0, 0), yaw 0°). Any run that doesn't start from spawn breaks this alignment.

**The map file is not part of this repository.** `navigation.launch.py`'s `map_file` argument defaults to an absolute path outside the repo (`~/ros2_ws/src/nav2_config/maps/farm_map.yaml`, a plain directory alongside `src/`, not a colcon package), and this repo's own `nav2_config/maps/` directory is tracked empty. Because git does not track empty directories, **a fresh clone's `nav2_config/maps/` won't exist at all, and `colcon build` will fail** on `nav2_config` with a `file INSTALL cannot find ... maps` error until you create it:

```bash
mkdir -p ~/ros2_ws/src/ros2-farm-scout-FYP/nav2_config/maps
```

To actually navigate, you additionally need a real map at the `map_file` path (or pass your own via `map_file:=...`) — either generate one yourself via the SLAM+`map_saver_cli` steps above, or copy one in.

## Mode Arbitration

The arbitration node (`mode_arbitration/mode_arbitration/arbitration_node.py`) implements:

- **MANUAL mode** (default): Forwards `/cmd_vel_teleop` to `/cmd_vel`
- **AUTO mode**: Forwards `/cmd_vel_auto` to `/cmd_vel`
- **Dead-man timeout**: If no `/cmd_vel_auto` received for 0.5 s in AUTO mode, the robot stops
- **Safe transitions**: Switching to MANUAL immediately publishes a stop command

Switch modes at runtime:
```bash
ros2 topic pub /mode std_msgs/msg/String "data: 'auto'" --once
ros2 topic pub /mode std_msgs/msg/String "data: 'manual'" --once
```

## Requirements

```bash
sudo apt install \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-joint-state-publisher
```

The virtual_maize_field package must be cloned into the workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/FieldRobotEvent/virtual_maize_field.git
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

On a fresh clone, create `nav2_config/maps/` first — see [Map & Localisation](#map--localisation) above, or the build will fail on that package.

## Running

### Simulation only (no navigation)
```bash
ros2 launch robot_bringup simulation.launch.py
```

### Full system (Nav2 navigation)
```bash
ros2 launch robot_bringup full_system.launch.py
```

Requires a map at the `map_file` path (see [Map & Localisation](#map--localisation)). Then set AUTO mode:
```bash
ros2 topic pub /mode std_msgs/msg/String "data: 'auto'" --once
```

### SLAM mapping mode (without Nav2)
```bash
ros2 launch robot_bringup full_system.launch.py use_slam:=true nav2:=false
```

### Run a scouting mission
```bash
python3 ~/ros2_ws/src/ros2-farm-scout-FYP/robot_bringup/scripts/scout_mission.py --auto --log trial_01
```

## Localisation

See [Map & Localisation](#map--localisation) above.

## Packages

| Package | Purpose |
|---|---|
| `farm_world` | SDF world file, Gazebo launch, maize/obstacle models |
| `robot_bringup` | Top-level launch files, scout mission, SLAM coverage driver |
| `nav2_config` | Nav2 parameters, costmap config (no map file ships with this package — see [Map & Localisation](#map--localisation)) |
| `mode_arbitration` | Arbitration node: routes `/cmd_vel_auto` or `/cmd_vel_teleop` → `/cmd_vel` |
| `teleop_node` | Keyboard teleoperation (w/a/s/d, publishes TwistStamped) |
| `robot_description` | Custom agricultural robot URDF (not used in current simulation) |

## Keyboard Controls

| Key | Action |
|---|---|
| `w` / `↑` | Forward |
| `s` / `↓` | Backward |
| `a` / `←` | Turn left |
| `d` / `→` | Turn right |
| `q` / `e` | Forward diagonal (left / right) |
| `z` / `c` | Backward diagonal (left / right) |
| `Space` / `x` | Stop |
| `k` | Quit |

Run in a real terminal — keyboard capture does not work in IDE output panes.

## Known Limitations

- **SLAM mapping has had reliability issues:** manual SLAM-driving sessions during development hit scan-match drift, a discrete `map→odom` pose-graph jump, and incomplete edge coverage. The saved-map + static-transform approach above sidesteps live SLAM/AMCL localisation for navigation entirely; it does not fix those underlying SLAM issues.
- **Odometry drift:** without AMCL correction, odometry accumulates error over long missions. Shorter waypoint segments (as used in `scout_mission.py`) mitigate this.
- **Static localisation assumes spawn:** the static `map→odom` transform is fixed to one specific pose. Missions must begin from spawn (-9.5, -10.0) facing north.
- **Mode switching is manual:** switching is triggered by publishing to `/mode`, not by automatic failure detection.
- **No dead-man timeout in MANUAL mode:** the 0.5 s AUTO-mode timeout has no MANUAL-mode equivalent — `arbitration_node` re-publishes the last `/cmd_vel_teleop` message indefinitely if nothing new arrives. `keyboard_teleop.py` protects against this itself (a 0.4 s local key-timeout that zeroes velocity), but any other publisher to `/cmd_vel_teleop` (e.g. a one-shot `ros2 topic pub`) must send its own explicit stop command.
- **The navigation map is not part of this repository** — see [Map & Localisation](#map--localisation). A fresh clone needs `nav2_config/maps/` created and a real map supplied before `full_system.launch.py`'s default configuration will build and navigate.
- **`robot_description` package:** contains a custom agricultural robot URDF that is not used in the current simulation (TurtleBot3 Waffle is used instead). Retained for potential future integration.

## License

Apache-2.0
