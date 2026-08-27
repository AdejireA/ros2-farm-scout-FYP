# ros2-farm-scout-FYP

Dual-mode agricultural field scouting robot. A Final Year Project built on ROS 2 Jazzy and Gazebo Harmonic.

This guide assumes you know the basics of ROS 2 (you've built a package, run `ros2 launch`, maybe published a topic by hand) but nothing about this specific project. Every command is given in full. Terms like "topic," "node," and "TF frame" are explained the first time they matter.

## 1. Project overview

A field scouting robot needs to drive itself through crop rows most of the time, but an operator has to be able to take control instantly if something goes wrong (an obstacle Nav2 didn't see, a person walking into the field, a sensor glitch). Handing control back and forth between an autonomous stack and a human, without ever letting both fight over the wheels at once, is a real safety problem, not just a convenience feature.

That handover is what this project actually contributes. The simulation world, the TurtleBot3 robot model, and the Nav2 navigation stack are all off-the-shelf components, wired together. The original piece of engineering is `mode_arbitration_node`: a small node that sits between Nav2's velocity output and a keyboard teleop node, decides which one is allowed to drive the robot at any given moment, and falls back to a stop if the autonomous side goes quiet. Everything else in this repo (the simulated field, the map, the evaluation scripts) exists to test that one node under realistic conditions.

End to end: Gazebo simulates a TurtleBot3 Waffle in a small maize field. Nav2 plans and drives routes through the crop rows using a pre-built map. A keyboard teleop node lets a person drive the same robot directly. `mode_arbitration_node` sits in between both of them and the robot's actual motor command, and a set of Python scripts run scripted missions against this whole stack to measure how well the handover holds up: how fast the robot stops when a human takes over, how fast it resumes when control is handed back, and how fast it stops on its own if Nav2 stops responding.

## 2. System architecture

A ROS 2 **package** is a self-contained unit of code and configuration (nodes, launch files, parameters) that gets built and installed together. This repo has six of them:

- **`farm_world`**: the Gazebo Harmonic simulation world: the field boundary, soil and grass ground planes, five crop rows, three obstacles, and the launch file that starts Gazebo with that world loaded.
- **`robot_description`**: a custom agricultural robot URDF (a description of a robot's links, joints, and sensors) that this project was originally designed around. It is not actually used in the simulation. The robot Gazebo spawns is a standard TurtleBot3 Waffle, loaded from ROS's own `turtlebot3_description`/`turtlebot3_gazebo` packages, not from anything in this repo. `robot_description`'s only active use is a standalone `display.launch.py` for inspecting that custom URDF in RViz.
- **`mode_arbitration`**: the FYP's actual contribution. One node, `arbitration_node`, described in detail below.
- **`teleop_node`**: `keyboard_teleop`, a node that reads w/a/s/d and arrow keys from the terminal and publishes velocity commands for manual driving.
- **`nav2_config`**: Nav2's parameter file (costmaps, planner, controller, SLAM Toolbox settings) and the directory where the navigation map lives.
- **`robot_bringup`**: the launch files that bring the whole system up together, plus the Python and shell scripts used to run and analyse evaluation missions.

**Topic flow.** A ROS 2 **topic** is a named channel that one or more nodes publish messages onto and one or more other nodes subscribe to. Nav2's controller publishes velocity commands to `/cmd_vel_auto`. `keyboard_teleop` publishes to `/cmd_vel_teleop`. Neither of these reaches the robot directly. `arbitration_node` subscribes to both, plus a `/mode` topic (a plain string, `"auto"` or `"manual"`), and republishes whichever one is currently selected to a single output topic, `/cmd_vel`, which is the only thing the Gazebo bridge actually listens to. The robot's default mode is MANUAL, so it starts under keyboard control until something switches it to AUTO.

Every velocity topic in this project uses `TwistStamped`, not the plain `Twist` message type older ROS tutorials often use. `TwistStamped` adds a header (a timestamp and frame name) to the same linear/angular velocity fields. The Gazebo Harmonic bridge used here expects the stamped version, so `Twist` would silently fail to connect.

**Dead-man timeout.** If the system is in AUTO mode and no `/cmd_vel_auto` message arrives for 0.5 seconds, `arbitration_node` assumes Nav2 has stalled or crashed and publishes a stop command on its own, without waiting for a human to notice. This timeout only applies in AUTO mode: MANUAL mode has no equivalent, so anything else that publishes to `/cmd_vel_teleop` (a script, a one-off `ros2 topic pub`) is responsible for sending its own stop.

**Localisation.** Nav2 normally figures out where the robot is on a saved map using AMCL (Adaptive Monte Carlo Localisation), a particle filter that matches live LIDAR scans against the map. This project doesn't use it. Instead, `navigation.launch.py` publishes one fixed **TF frame** transform (TF is ROS 2's system for tracking the position and orientation of one frame relative to another) from `map` to `odom`, anchored to the robot's known spawn pose. This works because the map itself is generated to align exactly with that spawn point (see the next section), so a fixed offset is all that's needed. The map is produced by `generate_farm_map.py`, which draws the occupancy grid directly from the field's known wall, crop-row, and obstacle coordinates rather than by driving the robot around with SLAM.

## 3. Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy (desktop install)
- Gazebo Harmonic
- Nav2
- TurtleBot3 packages
- Python 3, plus `numpy`, `Pillow`, and `matplotlib`

Install the ROS-side dependencies:

```bash
sudo apt install \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-joint-state-publisher
```

Install the Python dependencies used by the map generator and the analysis scripts (not needed just to run the simulation itself):

```bash
pip install numpy Pillow matplotlib --break-system-packages
```

(`--break-system-packages` is needed on Ubuntu 24.04's system Python. If you're using a virtual environment instead, drop that flag.)

## 4. Building from source

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url> ros2-farm-scout-FYP
git clone https://github.com/FieldRobotEvent/virtual_maize_field.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The crop plant models live in `virtual_maize_field`, cloned separately above; without it Gazebo can't find `model://maize_01`/`maize_02` and the world fails to load.

`nav2_config/maps/` used to be an empty, untracked folder, which meant `colcon build` failed on a fresh clone with a `file INSTALL cannot find ... maps` error, because git doesn't track empty directories. A `.gitkeep` placeholder file now keeps that directory present in every clone, so the build itself no longer fails. That placeholder does not include an actual map, though, since generated map data isn't committed to the repository. Generate one after building:

```bash
python3 ~/ros2_ws/src/ros2-farm-scout-FYP/robot_bringup/scripts/generate_farm_map.py
```

This writes `farm_map.pgm` and `farm_map.yaml` to `~/ros2_ws/src/nav2_config/maps/`, a plain directory alongside your workspace's `src/` folder, not inside the cloned repo. That's the exact path Nav2's `map_file` launch argument defaults to, so no further configuration is needed once this has been run once.

## 5. The simulation world

The field is defined in `farm_world/worlds/farm.sdf`: a 22 x 24 metre rectangle enclosed by walls at x = ±11 m and y = ±12 m.

- **Crop rows**: five rows at x = -8, -4, 0, 4, 8 m, each with 10 maize plants spaced 2 m apart along y, for 50 plants total.
- **Obstacles**: a rock at (-6, -4), a crate at (6, 3), and a narrow post at (0, -6).
- **Robot spawn**: (-9.5, -10.0), facing north (yaw = π/2 radians, 90 degrees).

Between each pair of crop rows sits a navigable gap, 2 m clear of the plants on either side, centred at x = -6, -2, 2, and 6. These four gap centrelines are the aisles used for pose calculations throughout the project. The scripted evaluation mission in `scout_mission.py`, though, only drives through two of them (x = -6 and x = -2) across a fixed 6-waypoint route: it's a repeatable test route for exercising the mode-arbitration logic, not a full sweep of the field.

## 6. Running the system

Bring up the full stack:

```bash
ros2 launch robot_bringup full_system.launch.py
```

This starts Gazebo with the field loaded, spawns the robot, starts the ROS-to-Gazebo bridge, brings up the full Nav2 stack, and starts `arbitration_node`. Four launch arguments control it:

| Argument | Default | What it does |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo's simulated clock instead of the wall clock. |
| `use_slam` | `false` | `true` runs SLAM Toolbox for live mapping instead of using the pre-built map and static transform. |
| `map_file` | (set to the generated map's path) | Absolute path to the map YAML used when `use_slam` is `false`. |
| `nav2` | `true` | `false` skips the entire Nav2 stack, useful when only mapping (with `use_slam:=true`) is needed. |

Override any of them on the command line, for example `ros2 launch robot_bringup full_system.launch.py use_slam:=true nav2:=false` for a SLAM-only, mapping-focused run.

To check what's actually running, a ROS 2 **node** is one running program in the system (`arbitration_node`, `bt_navigator`, and so on), and a **topic**, again, is a named message channel between nodes:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /current_mode
```

Switch between AUTO and MANUAL by publishing to `/mode`:

```bash
ros2 topic pub /mode std_msgs/msg/String "data: 'auto'" --once
ros2 topic pub /mode std_msgs/msg/String "data: 'manual'" --once
```

To drive by hand, run the teleop node in its own terminal (it only has an effect while the system is in MANUAL mode, which is the default):

```bash
ros2 run teleop_node keyboard_teleop
```

Controls: `w`/`↑` forward, `s`/`↓` backward, `a`/`←` turn left, `d`/`→` turn right, `q`/`e` diagonal forward, `z`/`c` diagonal backward, space or `x` to stop, `k` to quit. This has to run in an actual terminal window, not an IDE's output panel, since it reads raw keypresses.

## 7. Running the evaluation

There are three scenarios, each testing a different part of the mode-arbitration safety story:

- **baseline**: the robot drives the normal 6-waypoint route. After waypoint 2 and again after waypoint 4, the script itself switches the system to MANUAL, waits a moment, then switches back to AUTO. This measures **stop latency** (how long from the mode switch until the robot's velocity actually drops to zero) and **resume latency** (how long from switching back until the robot is moving again).
- **deadman**: the same route, but partway through waypoint 3 the script deliberately cancels Nav2's navigation goal, simulating Nav2 crashing or hanging. Nothing switches the mode this time; the point is to see whether `arbitration_node`'s own 0.5-second dead-man timeout notices the silence and stops the robot without being told to. That reaction time is the **dead-man response time**.
- **obstacle**: simulates an operator spotting something Nav2 missed and taking over manually mid-drive, then handing control back. It measures the same stop and resume latencies as baseline, but from a takeover the script triggers mid-goal rather than at a fixed waypoint boundary.

All three also log per-waypoint navigation outcome (succeeded, failed, or canceled) and how long each waypoint took.

Run a single trial:

```bash
python3 robot_bringup/scripts/scout_mission.py --scenario baseline --auto --log trial_results/baseline_01
```

`--scenario` picks one of the three above. `--auto` runs straight through without pausing for Enter between waypoints; leave it off to step through manually. `--log <name>` writes the trial's results to `<name>.csv`.

For a full batch, `robot_bringup/scripts/run_trials.sh` automates it:

```bash
./robot_bringup/scripts/run_trials.sh baseline 1 20
./robot_bringup/scripts/run_trials.sh deadman 1 20
./robot_bringup/scripts/run_trials.sh obstacle 1 20
```

This script kills and fully relaunches Gazebo, Nav2, and every other node before **every single trial**, not just between scenarios. That's slower, but it means no trial's measurements can be skewed by state left over from the one before it. If you're scripting trials yourself rather than using `run_trials.sh`, do the same, restart the whole system between runs, since a leftover mode setting or a Nav2 stack that's already mid-goal will throw off the latency numbers.

CSVs land in `trial_results/`, named `<scenario>_<NN>.csv`.

## 8. Analysing results

```bash
python3 robot_bringup/scripts/analyse_trials.py
```

Run with no arguments, it reads every CSV in `trial_results/` and writes output to `analysis_output/`. It prints five tables to the terminal (navigation success rate, mission completion time, stop latency, resume latency, and dead-man response time), and writes two CSVs (`all_trials_combined.csv`, `summary_stats.csv`) and four charts (`stop_latency_boxplot.png`, `resume_latency_boxplot.png`, `nav_success_rate.png`, `deadman_latency.png`) into that output directory.

## 9. Results summary

From the 60 completed trials (20 each of baseline, deadman, and obstacle):

- **80 out of 80** mode-arbitration tests (mode switches, dead-man triggers, and takeovers combined) succeeded, a 100% pass rate.
- **Stop latency**: 1.3 ms median (baseline scenario).
- **Resume latency**: 84.6 ms median in baseline, 125.5 ms median in obstacle.
- **Dead-man response time**: 39.2 ms median.
- **Overall navigation success rate**: 97.8% (352 of 360 waypoints reached across all three scenarios; baseline alone was 100%).

These figures come directly from `analysis_output/summary_stats.csv`.

## 10. Known limitations

- **No AMCL, odometry-only localisation.** Position is tracked by dead reckoning from the wheel odometry plus the fixed map-to-odom transform, with nothing correcting it against sensor data. Small errors accumulate the longer the robot drives (this is what "drift" means), which is part of why the evaluation route is kept short.
- **Live obstacle avoidance is disabled.** The costmaps' LIDAR-based obstacle layer was turned off during development, because the maize rows were interfering with sensor-based obstacle detection along the aisles. Obstacle avoidance in the current setup relies entirely on the pre-built static map rather than live scan data.
- **"Succeeded" means goal tolerance, not a verified collision-free path.** Nav2 reports a waypoint as succeeded once the robot is within its configured position and heading tolerance of the goal. It says nothing about what happened along the way.
- **The map is programmatic, not SLAM-built.** `generate_farm_map.py` draws the occupancy grid from known coordinates rather than from a real mapping run, which is fast and repeatable but means it will only ever be as accurate as the coordinates written into that script.
- **A shutdown race prints an error on exit.** `rclpy`'s shutdown sequence can trigger a "terminate called" message when `scout_mission.py` exits. It's cosmetic: it happens after the trial's data has already been written, and doesn't affect the logged results.

## 11. Repository structure

```
ros2-farm-scout-FYP/
├── farm_world/                       # Gazebo world: field, crop rows, obstacles
│   ├── worlds/farm.sdf               # The world definition itself
│   └── launch/gazebo.launch.py       # Launches Gazebo with this world only
├── robot_description/                # Custom robot URDF (not used by the simulation)
│   ├── urdf/agricultural_robot.urdf.xacro
│   └── launch/display.launch.py      # Standalone RViz viewer for that URDF
├── mode_arbitration/                 # The FYP contribution
│   └── mode_arbitration/arbitration_node.py
├── teleop_node/                      # Keyboard manual control
│   └── teleop_node/keyboard_teleop.py
├── nav2_config/                      # Nav2 parameters and map location
│   ├── config/nav2_params.yaml
│   └── maps/                         # Generated map goes here (see §4)
├── robot_bringup/                    # Launch files and all scripts
│   ├── launch/
│   │   ├── simulation.launch.py      # Gazebo + robot + bridge only
│   │   ├── navigation.launch.py      # Nav2 stack
│   │   └── full_system.launch.py     # Everything together
│   └── scripts/
│       ├── generate_farm_map.py      # Builds the map from known geometry
│       ├── scout_mission.py          # Runs one evaluation trial
│       ├── run_trials.sh             # Runs a batch of trials
│       ├── analyse_trials.py         # Summarises trial_results/ into analysis_output/
│       └── slam_coverage_drive.py    # Scripted SLAM-driving helper (not part of the main evaluation)
├── trial_results/                    # Raw per-trial CSVs (60 files)
├── analysis_output/                  # Tables, charts, and combined CSVs from analyse_trials.py
├── LICENSE
└── README.md
```

## 12. License

Apache-2.0. See `LICENSE`.
