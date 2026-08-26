# Repository Audit

Full pass over every tracked file in this repository, checked against the questions
below. Nothing was changed — this document is the only output. All findings were
verified directly (reading the file, or in a few cases running a read-only command
against a copy of the file/tree) rather than assumed; where a command was run, the
command and result are given inline.

Classification used throughout: **bug** (code/config does something other than what
it's supposed to, or silently does nothing), **stale comment** (text that no longer
matches the code beside it), **missing dependency** (something imported/launched that
isn't declared anywhere a fresh install would pick it up), **dead code/config** (present
in the file, never actually exercised by anything that runs).

---

## 1. Does the code match what the README says?

`README.md` was independently re-verified against the current code as part of writing
this audit (not assumed from an earlier pass). Two points stand out that the README
itself does not resolve, listed here so they're not lost:

| # | Issue | Where | Class |
|---|---|---|---|
| 1.1 | README's Requirements list (`ros-jazzy-turtlebot3`, `nav2-bringup`, `slam-toolbox`, `ros-gz`, `joint-state-publisher`) does not mention `tf2_ros`, `nav_msgs`, `numpy`, or `Pillow` — all four are real runtime requirements of scripts in this repo (see §5). | `README.md` "Requirements" | missing dependency (documentation-side) |
| 1.2 | Everything else checked in the README (topics, message types, spawn pose, field geometry, arbitration behaviour, map/localisation mechanism) matches the code as of this audit. | — | — (confirms, no finding) |

No contradictions were found between README's prose and the current behaviour of
`arbitration_node.py`, `keyboard_teleop.py`, `simulation.launch.py`, or the field
geometry in `farm.sdf`.

---

## 2. Stale comments, wrong constants, dead code

This is the largest category. Findings are grouped by file.

### `nav2_config/config/nav2_params.yaml`

| Line(s) | Issue | Class |
|---|---|---|
| 4–6 | File header comment: *"Two localisation modes are provided... `use_slam:=false` — AMCL localisation against a saved map (default)."* This is no longer true — `navigation.launch.py`'s non-SLAM branch publishes a static `map→odom` transform via `tf2_ros static_transform_publisher`, not AMCL. AMCL is never instantiated anywhere in this repo's current launch files. | stale comment |
| 14–53 | The entire `amcl:` parameter block (40 lines) configures a node (`nav2_amcl`) that no launch file in this repo instantiates. Dead config — kept current-looking (e.g. `laser_max_range: 3.5` matches the real sensor) but never loaded by anything that runs. | dead code |
| 302–315 | `global_costmap.obstacle_layer` is fully defined (topic `/scan`, ranges, etc.) but **is not listed in `global_costmap.plugins: [static_layer, inflation_layer]`** (line 298). Nav2's costmap framework only instantiates layers named in `plugins`; an unlisted block is inert. Net effect: the global costmap never incorporates live LIDAR data — only the static map + inflation. | dead code / bug (silent — nothing errors, it just never runs) |
| 261–283 | `local_costmap` has no `obstacle_layer` at all (only `static_layer` + `inflation_layer`, line 274). Combined with the point above, **neither costmap reacts to live `/scan` data** — obstacle avoidance is entirely dependent on obstacles already being baked into the static map (see §8/generate_farm_map.py). This may be intentional (the world is fully known and static), but it is not stated anywhere, and it directly affects what the `obstacle`-scenario trials in `trial_results/` are actually testing (a scripted mode-switch, not sensor-triggered avoidance). | behavioural gap, undocumented |
| 386–415 | `lifecycle_manager:` and `lifecycle_manager_slam:` blocks (30 lines) are also dead: `navigation.launch.py`'s `Node(package='nav2_lifecycle_manager', ...)` builds its `node_names` parameter **inline in Python** (`{'node_names': managed}`) and never includes `params_file` in that node's `parameters=[...]` list, so this YAML section is never read. Doubly stale since its `node_names` list (line 392) still includes `amcl`. | dead code |

### `mode_arbitration/mode_arbitration/arbitration_node.py`

| Line(s) | Issue | Class |
|---|---|---|
| 6, 7, 10 | Module docstring types the three velocity topics as `(geometry_msgs/Twist)`. The code three lines below imports and uses `TwistStamped` exclusively on all three topics (`/cmd_vel_auto`, `/cmd_vel_teleop`, `/cmd_vel`). No functional issue — the docstring just contradicts its own file. | stale comment |

### `robot_bringup/launch/full_system.launch.py`

| Line(s) | Issue | Class |
|---|---|---|
| 7–8 | Docstring: *"3. Nav2 stack (SLAM or AMCL mode)"*. Same stale-AMCL issue as above. | stale comment |
| 13–14 | Docstring: *"map_file (default '') — path to map YAML used with AMCL"*. The actual default (line 107) is `/home/ssrlserg1/ros2_ws/src/nav2_config/maps/farm_map.yaml`, not `''`, and "used with AMCL" is stale for the same reason as above. | stale comment |
| 59 vs 106–108 | `map_file = LaunchConfiguration('map_file', default='')` (line 59) declares an inline fallback of `''`, but the file's own `DeclareLaunchArgument('map_file', default_value='/home/ssrlserg1/.../farm_map.yaml', ...)` three lines later (106–108) is what actually takes effect for any normal invocation. The two defaults for the same argument, in the same file, disagree — harmless in practice (`DeclareLaunchArgument` wins), but genuinely misleading to read. | dead code / stale comment |

### `robot_bringup/scripts/scout_mission.py.bak`, `robot_bringup/scripts/slam_coverage_drive.py.bak`

| Issue | Class |
|---|---|
| Both `.bak` files are tracked by git (added in commits `636606c` and `c1dfffe`, alongside the rewrites that superseded them) and are meaningfully different, older versions of the live scripts — e.g. `slam_coverage_drive.py.bak` has no "wait for map frame" startup guard that the current file has, and different speed/timeout constants. `.gitignore` does not exclude `*.bak`. These are pure leftover artifacts: version control is already the backup mechanism, so committing hand-made backup copies alongside the real file just leaves two versions of the truth sitting in the tree, one of which nothing points at. | dead code (tracked, unused) |

### `robot_description/package.xml`

| Line | Issue | Class |
|---|---|---|
| 11 | Declares `<exec_depend>joint_state_publisher</exec_depend>`, but the only launch file in this package, `display.launch.py` (line 24), instantiates `package='joint_state_publisher_gui'` — a different, separate ROS package. `joint_state_publisher_gui` is not declared anywhere. | bug / missing dependency |

---

## 3. Do launch file arguments match their defaults and descriptions?

| File | Argument | Declared default | Description text | Match? |
|---|---|---|---|---|
| `simulation.launch.py` | `use_sim_time` | `'true'` | (none) | consistent |
| `navigation.launch.py` | `use_sim_time` | `'true'` | "Use /clock from simulation" | consistent |
| `navigation.launch.py` | `use_slam` | `'false'` | "true=SLAM Toolbox, false=AMCL+map" | **stale** — false no longer means AMCL, it means the static-transform path (see §2) |
| `navigation.launch.py` | `map_file` | `/home/ssrlserg1/ros2_ws/src/nav2_config/maps/farm_map.yaml` | "Absolute path to map YAML for AMCL mode" | **stale** ("AMCL mode") + the default is a machine-specific absolute path outside the repo (see §4, §8) |
| `navigation.launch.py` | `nav2` | `'true'` | "false = skip the full Nav2 stack, slam_toolbox only" | consistent |
| `full_system.launch.py` | `use_sim_time`, `use_slam`, `nav2` | match `navigation.launch.py`'s defaults exactly | — | consistent between the two files |
| `full_system.launch.py` | `map_file` | same absolute path as `navigation.launch.py`'s default | "Absolute path to map YAML (AMCL mode)" | same staleness as above; at least the two files now **agree** with each other on the actual default value (they did not always — see `PROJECT_STATUS.md`/git history) |
| `farm_world/gazebo.launch.py` | `gz_args` | `['-r ', world_file]` | "Arguments passed to gz sim" | consistent |

No argument default was found to silently diverge from its own file's description in a
way that would change behaviour unexpectedly — the "AMCL" wording is stale text, not a
functional mismatch, and the one duplicate-default oddity (§2) resolves to the correct
value regardless.

---

## 4. File path consistency (launch files, scripts, configs)

| Path | Used in | Consistent? |
|---|---|---|
| `map://` field constants (`WALL_X=11`, `WALL_Y=12`, spawn `(-9.5,-10)`, aisle centerlines `[-6,-2,2,6]`) | `farm_world/worlds/farm.sdf`, `robot_bringup/scripts/slam_coverage_drive.py`, `robot_bringup/scripts/scout_mission.py`, `robot_bringup/scripts/generate_farm_map.py` | **Yes** — every one of these files' hardcoded geometry constants was cross-checked against `farm.sdf`'s actual `<pose>`/`<size>` values directly; all agree. |
| `map_file` default path | `navigation.launch.py`, `full_system.launch.py` | **Yes**, the two agree with each other. **No**, in a broader sense — it's `/home/ssrlserg1/ros2_ws/src/nav2_config/maps/farm_map.yaml`, a machine-specific absolute path baked into the launch file, pointing *outside* this git repository entirely (a plain, untracked directory at the workspace root, not a colcon package). It will not resolve on any machine where the user's home directory isn't literally `ssrlserg1`. Confirmed this directory is genuinely outside the repo: `git ls-files` returns nothing under it. |
| `generate_farm_map.py`'s default output directory | `robot_bringup/scripts/generate_farm_map.py:56` | Matches the `map_file` default above exactly (`~/ros2_ws/src/nav2_config/maps`) — consistent with each other, but **not** with the repo's own `nav2_config/maps/` (see §8), so running the generator with no arguments does not populate the package's own tracked location. |
| TurtleBot3 URDF path | `simulation.launch.py:37`, hardcoded `/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf` | Internally consistent (only referenced once), but it is an absolute path into the ROS install tree rather than resolved via `FindPackageShare('turtlebot3_description')`. Works today because `ros-jazzy-turtlebot3` installs to a fixed, predictable location, but it's a hardcoded distro-specific path rather than a portable substitution, unlike every other package reference in this repo (`FindPackageShare(...)` is used everywhere else, e.g. `display.launch.py`, `farm_world/gazebo.launch.py`). |
| `GZ_SIM_RESOURCE_PATH` | Set in `simulation.launch.py` (both `turtlebot3_gazebo/models` and `virtual_maize_field/models`); **not set** in `farm_world/gazebo.launch.py` | Launching `farm_world/gazebo.launch.py` on its own fails to resolve `model://maize_01`/`maize_02` — confirmed by direct testing in an earlier phase of this project. `simulation.launch.py` (which includes the world differently) works because it sets the variable itself. This is a structural gap between the two launch files, not a typo. |

---

## 5. Does `package.xml` list all actual dependencies?

Checked by cross-referencing every `import`/`from` in every `.py` file and every
`package='...'` in every launch file against that package's own `<exec_depend>` list.

| Package | Missing dependency | Actually used by | Class |
|---|---|---|---|
| `robot_bringup` | `tf2_ros` | `slam_coverage_drive.py:100–101` (`from tf2_ros import Buffer, TransformListener, ...`) **and** `navigation.launch.py:212` (`package='tf2_ros', executable='static_transform_publisher'`) — two independent, unrelated uses, neither declared | missing dependency |
| `robot_bringup` | `nav_msgs` | `slam_coverage_drive.py:339` (`from nav_msgs.srv import GetMap`, inside `check_full_coverage()`) | missing dependency |
| `robot_bringup` | `numpy`, `Pillow` (no ROS package name — plain Python libs) | `generate_farm_map.py:16–17` | missing dependency (not expressible as a normal `exec_depend`, but undocumented anywhere in the package) |
| `robot_bringup` | `matplotlib` (optional) | `analyse_trials.py:19–22`, guarded by a `try/except ImportError` with a helpful message | **not a bug** — the script degrades gracefully and tells the user how to install it; noted for completeness only |
| `nav2_config` | `nav2_smoother` | `navigation.launch.py:104` (`package='nav2_smoother', executable='smoother_server'`), instantiated in **every** Nav2-enabled configuration (SLAM and non-SLAM alike) | missing dependency |
| `robot_description` | `joint_state_publisher_gui` (declares `joint_state_publisher` instead — see §2) | `display.launch.py:24` | missing dependency / bug |

`nav2_amcl` remains declared in `nav2_config/package.xml` even though nothing launches
it any more (see §2) — not "missing," the opposite: a dependency kept around after its
only user was removed. Harmless, but worth noting alongside the above.

---

## 6. Does `CMakeLists.txt` install everything it should?

| Package | CMakeLists installs | Actually present / needed | Gap |
|---|---|---|---|
| `robot_bringup` | `launch/`, `scripts/scout_mission.py` → `scout_mission`, `scripts/slam_coverage_drive.py` → `slam_coverage_drive` | `scripts/` also contains `generate_farm_map.py` and `analyse_trials.py` | Neither is installed as an executable. Both are still runnable directly with `python3 <path>` (both have `#!/usr/bin/env python3` shebangs and are plain scripts, not dependent on the install step for imports), but neither gets a `ros2 run robot_bringup <name>` entry point the way the other two scripts do. Also installs the two `.bak` files' *directory* implicitly if ever referenced — it isn't, they're just untouched by CMake, sitting in `scripts/` doing nothing. |
| `nav2_config` | `config/`, `maps/` → `share/nav2_config` | `maps/` exists on disk here but is **empty and untracked by git** | **Critical** — see §8. `install(DIRECTORY ...)` requires the source directory to exist; on a fresh clone it won't. Verified directly: archived the current commit (`git archive HEAD`) into a scratch directory and ran `cmake` + `make install` against just this package — configure succeeds, but `make install` fails with `CMake Error ... file INSTALL cannot find ".../nav2_config/maps": No such file or directory.` |
| `farm_world` | `worlds/`, `launch/` | matches actual directory contents | none |
| `robot_description` | `urdf/`, `launch/` | matches actual directory contents | none |
| `mode_arbitration`, `teleop_node` (ament_python, `setup.py`) | single console-script entry point each, matching their one node file | matches | none |

---

## 7. Files not tracked by git that should be

| Finding | Class |
|---|---|
| **No `LICENSE` file anywhere in the repository**, despite all six `package.xml` files declaring `<license>Apache-2.0</license>` and `README.md`'s final line stating "Apache-2.0". A real Apache-2.0 grant requires the license text and a NOTICE file to actually apply; right now the license is asserted in five places and present in none. | missing file (administrative, not code) |
| `nav2_config/maps/farm_map.yaml`/`.pgm` — the file the default `map_file` argument points to — exists only outside the repo (`~/ros2_ws/src/nav2_config/`, a bare directory, not a package) and is not part of this git history at all, tracked or otherwise. Already covered in depth in §4/§8 and in `README.md`. | missing file (by design, per README, but worth restating here since it's the direct answer to this question) |

Conversely — checked for the opposite problem too (files that exist on disk here but
are `.gitignore`d and maybe shouldn't be): `.gitignore`'s `nav2_config/maps/*.pgm` line
only excludes the raster image, not the paired `.yaml`, and neither file exists in this
package's tracked `maps/` directory regardless, so this doesn't currently hide anything.
`PHASE*.md` is also gitignored, which is why `PHASE0_CURRENT_STATE.md`,
`PHASE1_RUNTIME_STABILITY.md`, and `PHASE2A_SLAM_READINESS.md` from earlier work in this
repository don't show up in `git status` — intentional per that gitignore rule, not a
gap.

---

## 8. Do the trial CSVs in `trial_results/` match the analysis output?

Verified by actually running the analysis rather than reading both sides and comparing
by eye: `robot_bringup/scripts/analyse_trials.py trial_results <scratch-dir>` was run
against the real, tracked `trial_results/` directory, writing to a throwaway directory
outside the repo, then diffed byte-for-byte against the tracked `analysis_output/`
files.

```
diff analysis_output/summary_stats.csv       <scratch>/summary_stats.csv        → identical
diff analysis_output/all_trials_combined.csv <scratch>/all_trials_combined.csv  → identical
```

**Result: match, exactly.** `analysis_output/summary_stats.csv` and
`all_trials_combined.csv` are exactly reproducible from the 15 CSVs currently in
`trial_results/` (5 `baseline_*`, 5 `deadman_*`, 5 `obstacle_*`) via the current version
of `analyse_trials.py`. All four `.png` charts regenerated without error as well
(not byte-compared, since chart rendering isn't guaranteed byte-stable across runs, but
the underlying numbers they're built from are the CSVs just confirmed identical above).

One path-related gap while checking this: `scout_mission.py` writes its `--log` output
to `~/ros2_ws/<name>.csv` (line 406 — outside both the repo and `trial_results/`), while
`analyse_trials.py` reads from `trial_results/` by default. Nothing in this repository
moves a file from the first location to the second — whoever produced the 15 files
currently in `trial_results/` did that copy step by hand, and no script or README
instruction documents it.

---

## Summary — Critical vs. Non-Critical

**Critical (will break a fresh `git clone` + `colcon build`):**

1. `nav2_config/CMakeLists.txt`'s `install(DIRECTORY config maps ...)` (§6) — `maps/` is
   empty and untracked, so it doesn't exist at all in a fresh clone. `make install`
   fails on this package specifically, with the exact error reproduced in §6/§8. This
   already has a documented one-line workaround in `README.md`
   (`mkdir -p nav2_config/maps` before building), but the underlying repo state is still
   broken for anyone who clones without having read that note first.

**Functional, but does not break the build (silent behavioural gaps — not cosmetic):**

2. `global_costmap`'s `obstacle_layer` is configured but not wired into `plugins:`, and
   `local_costmap` has no obstacle layer at all (§2) — Nav2 will run fine, but neither
   costmap reacts to live `/scan` data; obstacle avoidance is entirely a function of
   what's already baked into the static map.
3. Five real missing/wrong dependency declarations (§5): `tf2_ros`, `nav_msgs` in
   `robot_bringup`; `nav2_smoother` in `nav2_config`; `joint_state_publisher_gui` vs. the
   wrongly-declared `joint_state_publisher` in `robot_description`. None of these break
   `colcon build` today (the packages happen to already be present transitively via the
   wider ROS/Nav2 install), but a minimal `rosdep`-driven install following only this
   repo's own declared dependencies would not pull them in.
4. `map_file`'s default is a machine-specific absolute path outside the repository
   (§4/§8) — works today on this machine, would not resolve for any other user/clone
   without either overriding `map_file` or recreating that exact external directory.

**Minor (cosmetic / documentation drift, no behavioural effect):**

5. Stale "AMCL" wording in five places across `nav2_params.yaml`'s header,
   `arbitration_node.py`'s docstring, both launch files' docstrings/argument
   descriptions, and `nav2_config/package.xml`'s `<description>` (§2, §3) — the actual
   mechanism is a static `map→odom` transform.
6. `teleop_node/package.xml`'s `<description>` says "publishes Twist" — code publishes
   `TwistStamped` (§5-adjacent, caught via §1).
7. Dead YAML configuration: the entire `amcl:` block and the `lifecycle_manager:`/
   `lifecycle_manager_slam:` blocks in `nav2_params.yaml` (§2) — neither is loaded by
   anything that runs.
8. Two tracked `.bak` files (`scout_mission.py.bak`, `slam_coverage_drive.py.bak`) that
   duplicate superseded versions of live scripts already preserved in git history (§2).
9. A duplicate, disagreeing default for `map_file` within `full_system.launch.py`
   itself (line 59 vs. 106–108) — harmless (the real default wins) but misleading to
   read (§2).
10. No `LICENSE` file despite six `package.xml`s and the README asserting Apache-2.0
    (§7).
11. `generate_farm_map.py` and `analyse_trials.py` are not installed by
    `robot_bringup/CMakeLists.txt`, so they have no `ros2 run` entry point unlike the
    other two scripts in the same directory (§6) — both remain runnable via
    `python3 <path>`, which is how `README.md` already documents them.
