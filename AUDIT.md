# Repository Audit (v2)

Fresh, full re-audit. The previous `AUDIT.md` is superseded by this file — several of
its findings have since been fixed and are marked **RESOLVED** below with how each was
verified. Everything else was re-checked directly against the current code, not carried
over from the old document. Nothing was changed as part of this audit; this file is the
only output.

Severity used throughout, per the request: **CRITICAL** (breaks a fresh clone/build),
**FUNCTIONAL** (runs and builds fine, but the behaviour is silently wrong, missing, or
not portable), **MINOR** (cosmetic — stale comments, wording, documentation gaps).

---

## RESOLVED since the previous audit

| # | Previous finding | Fix commit(s) | Verified how |
|---|---|---|---|
| R1 | `nav2_config/CMakeLists.txt`'s `install(DIRECTORY config maps ...)` failed on a fresh clone because `maps/` was empty and untracked. | `c76bdb7` (`.gitkeep` added) | **Re-ran the exact fresh-clone simulation from the last audit**: `git archive HEAD` into a scratch tree, `cmake` + `make install` against just `nav2_config`. This time it completes cleanly — `maps/.gitkeep` installs alongside `config/nav2_params.yaml`. No error. |
| R2 | `robot_bringup/package.xml` missing `tf2_ros`, `nav_msgs` (used by `slam_coverage_drive.py` and `navigation.launch.py`'s `static_transform_publisher`). | `048a3ca` | Read `robot_bringup/package.xml` — both now present. |
| R3 | `nav2_config/package.xml` missing `nav2_smoother` (used by `navigation.launch.py`'s `smoother_server`). | `048a3ca` | Read `nav2_config/package.xml` — present. |
| R4 | `robot_description/package.xml` declared `joint_state_publisher`, but `display.launch.py` uses `joint_state_publisher_gui`. | `048a3ca` | Read `robot_description/package.xml` — now declares `joint_state_publisher_gui`. |
| R5 | `full_system.launch.py` line 59 had a duplicate, disagreeing default (`LaunchConfiguration('map_file', default='')` vs. the real `DeclareLaunchArgument` default). | `978c25d` | Read the file — line now reads `LaunchConfiguration('map_file')`, no inline default. |
| R6 | Stale "AMCL" wording in `nav2_params.yaml`'s header comment, `full_system.launch.py`'s docstring and `DeclareLaunchArgument` descriptions, `navigation.launch.py`'s module docstring and two inline comments. | `94d2fcc`, plus a follow-up round | Re-read all four files; every one of the specifically-flagged lines now describes the static `map→odom` transform instead of AMCL. (Two AMCL mentions remain outside what was ever asked to be fixed — see §2 below, not treated as newly broken.) |
| R7 | `arbitration_node.py`'s module docstring typed its topics as `geometry_msgs/Twist`; the code uses `TwistStamped`. | `683c264` | Read the docstring — now says `TwistStamped` in all three places. |
| R8 | `scout_mission.py.bak` / `slam_coverage_drive.py.bak` tracked in git, duplicating superseded script versions. | `84a8f50` / `7ff9052` | `git ls-files` no longer lists either `.bak` file (history preserved in the commits that removed them, per the commit message). |
| R9 | No `LICENSE` file despite six `package.xml`s and the README claiming Apache-2.0. | `6e3e79f` | `LICENSE` now present at repo root — standard, unmodified Apache-2.0 text (202 lines), matches every package's `<license>Apache-2.0</license>` tag and the README. See §10. |

The `amcl:` YAML block and the dead `lifecycle_manager:`/`lifecycle_manager_slam:`
blocks in `nav2_params.yaml` were explicitly left untouched by instruction during the
AMCL-wording fix — confirmed still present, byte-identical in substance, still dead.
Not a regression; see §2.

---

## 1. Does the code match what the README says?

Re-checked in full against the current code, not the previous audit's notes.

| Finding | Class | Severity |
|---|---|---|
| README §"Scouting Mission" (line 62) says `--log <name>` results are written to `~/ros2_ws/<name>.csv`. That's only true when `<name>` has no `/` in it. `scout_mission.py` (lines 405–409) now branches: if `log_name` contains `/`, it writes to that path directly instead. This is exactly how `run_trials.sh` calls it (`--log "$TRIAL_DIR/$trial_name"`, an absolute path) — the actual mechanism the 60 tracked trial CSVs came from isn't the one path the README describes. | stale/incomplete doc | MINOR |
| README never mentions `run_trials.sh` or `analyse_trials.py` anywhere — both exist, both work (see §9), neither is documented. A reader following the README alone would not discover how the 60 files in `trial_results/` or the contents of `analysis_output/` were produced. | missing documentation | MINOR |
| Everything else checked — topics/message types, field geometry, arbitration behaviour, robot specs, Map & Localisation mechanism, Known Limitations — matches the current code. | — | — (confirms, no finding) |

---

## 2. Stale comments, wrong constants, dead code

### `nav2_config/config/nav2_params.yaml` — unchanged from previous audit, confirmed still present

| Line(s) | Issue | Class | Severity |
|---|---|---|---|
| 14–52 | `amcl:` block (39 lines) configures a node nothing launches. Left in place on instruction as dead config. | dead code | MINOR |
| 300–313 | `global_costmap.obstacle_layer` is fully defined (scan topic, ranges) but **not listed** in `global_costmap.plugins: [static_layer, inflation_layer]` (line 297). Never instantiated. | dead code / silent behavioural gap | **FUNCTIONAL** |
| 261–283 (local_costmap block) | `local_costmap` has no `obstacle_layer` at all. Combined with the point above, **neither costmap reacts to live `/scan` data** — obstacle avoidance depends entirely on what's already baked into the static map. Unchanged from previous audit; still undocumented anywhere as an intentional design choice. | behavioural gap | **FUNCTIONAL** |
| 386–415 | `lifecycle_manager:`/`lifecycle_manager_slam:` blocks (30 lines) are dead — `navigation.launch.py` builds `node_names` inline in Python and never loads `params_file` for that node. Still lists `amcl` in `node_names` (line 394), which would be stale even if the block were read. | dead code | MINOR |

### `robot_bringup/scripts/scout_mission.py` — new finding this pass

| Line | Issue | Class | Severity |
|---|---|---|---|
| 422 | `#nav.lifecycleShutdown()` — commented out, not removed. Unclear whether this is a deliberate workaround (e.g. it previously hung or errored) or a leftover debug change. Either way: if unintentional, Nav2's lifecycle nodes may not be told to deactivate/shut down cleanly when the script exits; if intentional, the line should be deleted rather than commented, since a `.bak` file was already removed elsewhere in this repo for exactly this kind of leftover. | dead code (commented-out, undocumented reason) | MINOR |

### `robot_bringup/launch/full_system.launch.py` / `navigation.launch.py` — two AMCL mentions outside the fixed scope

| Line | Issue | Class | Severity |
|---|---|---|---|
| `full_system.launch.py:17` | `touches Nav2/AMCL — only slam_toolbox comes up)` — inside the Quick-start docstring, never included in either round of AMCL-wording fixes. Same staleness as the now-fixed lines around it. | stale comment | MINOR |
| `navigation.launch.py:208` | `# Static map->odom transform (replaces AMCL for simulation).` — this one is accurate (explains history correctly), not stale. Listed here only for completeness since it matched a repo-wide "AMCL" search. | — | — (not a finding) |

### `nav2_config/package.xml`, `teleop_node/package.xml` — descriptions never touched by the dependency-fix commit

| File:Line | Issue | Class | Severity |
|---|---|---|---|
| `nav2_config/package.xml:6` | `<description>Nav2 configuration for agricultural robot: AMCL localisation and DWB controller</description>` — "AMCL localisation" is stale (DWB is still accurate). The `048a3ca` commit added the missing `nav2_smoother` dependency but did not touch this description. | stale comment | MINOR |
| `teleop_node/package.xml:6` | `<description>Keyboard teleoperation node — publishes Twist to /cmd_vel_teleop</description>` — code publishes `TwistStamped`. Never fixed (the dependency-fix commit only added/corrected `<exec_depend>` entries, not descriptions). | stale comment | MINOR |
| `nav2_config/package.xml:9` | Still declares `<exec_depend>nav2_amcl</exec_depend>` even though nothing launches `nav2_amcl` any more. Not "missing," the opposite — a dependency that outlived its only user. Harmless but worth noting alongside the two points above; all three are the same underlying "AMCL was removed from the code but not from its surrounding metadata" pattern. | dead dependency | MINOR |

---

## 3. Do launch file arguments match their defaults and descriptions?

Re-verified against the current files.

| File | Argument | Default | Description | Match? |
|---|---|---|---|---|
| `simulation.launch.py` | `use_sim_time` | `'true'` | (none) | consistent |
| `navigation.launch.py` | `use_sim_time`, `nav2` | unchanged | unchanged | consistent |
| `navigation.launch.py` | `use_slam` | `'false'` | now "true=SLAM Toolbox, false=static map->odom transform + saved map" | **consistent** (RESOLVED, see §R6) |
| `navigation.launch.py` | `map_file` | `/home/ssrlserg1/ros2_ws/src/nav2_config/maps/farm_map.yaml` | now "Absolute path to map YAML for the static-transform localisation mode" | consistent wording; the default itself is still a machine-specific absolute path outside the repo — not a wording problem, see §4 |
| `full_system.launch.py` | `use_slam`, `map_file`, `nav2` | match `navigation.launch.py` | now static-transform wording throughout | consistent (RESOLVED) |
| `farm_world/gazebo.launch.py` | `gz_args` | `['-r ', world_file]` | "Arguments passed to gz sim" | consistent |

No remaining argument was found where the default value and its description text
actively disagree in a way that would mislead about *behaviour* (as opposed to the
purely cosmetic AMCL wording already covered in §2).

---

## 4. File path consistency (launch files, scripts, configs)

| Path | Used in | Consistent? |
|---|---|---|
| Field geometry constants (`WALL_X`, `WALL_Y`, spawn, aisles) | `farm.sdf`, `slam_coverage_drive.py`, `scout_mission.py`, `generate_farm_map.py` | Still consistent — unchanged since the previous audit, re-confirmed. |
| `map_file` default | `navigation.launch.py`, `full_system.launch.py` | Agree with each other; still a machine-specific absolute path (`/home/ssrlserg1/...`) outside the repository. Unchanged from previous audit — **FUNCTIONAL**, works today, not portable to another user/clone without override. |
| **New**: `robot_bringup/scripts/run_trials.sh` lines 13–14 | `SCOUT_SCRIPT="$HOME/ros2_ws/src/ros2-farm-scout-FYP/robot_bringup/scripts/scout_mission.py"`, `TRIAL_DIR="$HOME/ros2_ws/src/ros2-farm-scout-FYP/trial_results"` | Both hardcode the exact clone path (`~/ros2_ws/src/ros2-farm-scout-FYP`) rather than resolving it relative to the script's own location or via `ros2 pkg prefix`. Works today, on this machine, with this exact directory name — would break for anyone who clones this repo under a different name or path. Same category of issue as `map_file`, not previously flagged because the script didn't exist yet. | **FUNCTIONAL** |
| `generate_farm_map.py`'s default output dir | matches `map_file`'s default exactly | consistent (unchanged, previously noted) |
| `GZ_SIM_RESOURCE_PATH` | set in `simulation.launch.py`, not in `farm_world/gazebo.launch.py` | unchanged structural gap from previous audit |

---

## 5. Does `package.xml` list all actual dependencies?

| Package | Status | Note |
|---|---|---|
| `robot_bringup` | `tf2_ros`, `nav_msgs` now declared. | RESOLVED (R2) |
| `nav2_config` | `nav2_smoother` now declared. `nav2_amcl` still declared despite being unused (§2) — not a missing-dependency problem, the inverse of one. | RESOLVED (R3), one stale leftover noted separately |
| `robot_description` | `joint_state_publisher_gui` now correctly declared in place of `joint_state_publisher`. | RESOLVED (R4) |
| `robot_bringup` | `numpy`, `Pillow` (used by `generate_farm_map.py`) still not declared anywhere (not expressible as a normal ROS `exec_depend`, but still undocumented). `matplotlib` (used by `analyse_trials.py`) remains optional and gracefully guarded — not a problem. | missing dependency (docs-level) | MINOR, unchanged |

No new missing-dependency findings this pass — `run_trials.sh` is a plain bash script
with no package dependency of its own beyond the ROS tools it shells out to, all of
which are already covered by `robot_bringup`'s and `nav2_config`'s existing dependency
lists.

---

## 6. Does `CMakeLists.txt` install everything it should?

| Package | Gap | Severity |
|---|---|---|
| `nav2_config` | RESOLVED — `maps/` now installs cleanly on a fresh clone (R1, re-verified live this pass). | — |
| `robot_bringup` | Still installs only `scout_mission.py` and `slam_coverage_drive.py` as `ros2 run`-able executables. `generate_farm_map.py` and `analyse_trials.py` remain uninstalled (unchanged from previous audit), and the **new** `run_trials.sh` is a third script in the same `scripts/` directory that also has no install rule. All three remain runnable directly (`python3 <path>` / `bash run_trials.sh`, both documented or self-evident), so this doesn't block anything — just an inconsistency: two of five scripts in that directory get a `ros2 run` entry point, three don't. | MINOR |
| `farm_world`, `robot_description` | Match actual directory contents. | — |
| `mode_arbitration`, `teleop_node` | Match their one console-script entry point each. | — |

Minor, unrelated observation while in this area: `scout_mission.py` and
`analyse_trials.py` are not marked executable (`644`, vs. `755` for
`generate_farm_map.py`, `slam_coverage_drive.py`, and `run_trials.sh`). Doesn't matter
in practice since every documented invocation explicitly prefixes `python3`, but it's
an inconsistency within the same directory. INFORMATIONAL, not scored.

---

## 7. Files not tracked by git that should be

| Finding | Status |
|---|---|
| No `LICENSE` file. | **RESOLVED (R9)** — present, correct, standard Apache-2.0 text. |
| `nav2_config/maps/` had no tracked content, so it wouldn't exist on a fresh clone. | **RESOLVED (R1)** — `.gitkeep` now tracked. |
| The actual `farm_map.yaml`/`.pgm` files (`map_file`'s default target) still live only outside the repo, at `~/ros2_ws/src/nav2_config/`, a bare directory that is not this repository and not under version control at all. This is by design per the README's own Map & Localisation section (`generate_farm_map.py` is meant to be run to produce them), not an oversight — restated here only because it's the direct answer to this checklist item. | unchanged, documented, not a new gap |

Checked the reverse direction too: nothing currently gitignored (`__pycache__/`,
`*.pyc`, `nav2_config/maps/*.pgm`, `PHASE*.md`, `virtual_maize_field/`, `.vscode/`) is
hiding anything that should actually be tracked.

---

## 8. Do the trial CSVs in `trial_results/` match the analysis output?

The dataset has grown since the previous audit — **60 trials now** (20 each of
`baseline`, `deadman`, `obstacle`; was 5 each / 15 total). Re-verified the same way as
before: ran `analyse_trials.py trial_results <scratch-dir>` fresh, against the real,
current `trial_results/` (60 files), and diffed byte-for-byte against the tracked
`analysis_output/` files.

```
diff analysis_output/summary_stats.csv       <scratch>/summary_stats.csv        → identical
diff analysis_output/all_trials_combined.csv <scratch>/all_trials_combined.csv  → identical
```

**Result: still an exact match.** All four charts regenerate without error. No finding
here — this is the second time in a row this pipeline has proven itself exactly
reproducible from its own tracked inputs.

---

## 9. Do `scout_mission.py`, `analyse_trials.py`, and `run_trials.sh` have correct paths and work as documented?

| Script | Check | Result |
|---|---|---|
| `scout_mission.py` | CSV output path handling for both a bare `--log name` and an absolute `--log /path/to/name` (as `run_trials.sh` passes) | Both work correctly (lines 405–409) — see §1 for the one place the README undersells this (only documents the bare-name case). |
| `scout_mission.py` | `--scenario`/`--auto`/`--log` argument parsing matches its own docstring (lines 15–18) | Matches exactly. |
| `scout_mission.py` | Requires `bt_navigator` + `map_server` active (`waitUntilNav2Active`, line 170) | Matches README's statement that it needs the default (non-SLAM) launch configuration. |
| `analyse_trials.py` | Default paths (`trial_results`, `analysis_output`), both relative to CWD | Correct when run from the repo root, as its own docstring specifies; not documented in README at all (§1). |
| `analyse_trials.py` | Reproduces the tracked `analysis_output/` exactly from the tracked `trial_results/` | Confirmed, §8. |
| `run_trials.sh` | Orchestrates `full_system.launch.py` (default args) → waits for `bt_navigator` → runs `scout_mission.py --scenario ... --auto --log <TRIAL_DIR>/<name>` → kills everything → repeats | Traced end-to-end; the sequence is internally consistent and matches how the 60 tracked CSVs were almost certainly produced. |
| `run_trials.sh` | Hardcoded absolute paths (lines 13–14) | Works today, not portable — §4, **FUNCTIONAL**. |
| `run_trials.sh` | Kill list (`kill_all()`, line 23) includes `static_transform_publisher` | Correctly updated for the current architecture — no leftover reference to `amcl`, unlike some of the comment/description drift found in §2. |

No functional bug was found in any of the three scripts beyond the path-portability
point already covered in §4, and the one commented-out line in `scout_mission.py`
covered in §2.

---

## 10. Is the LICENSE file present and correct?

**Yes — RESOLVED.** `LICENSE` (202 lines) is present at the repo root and is the
complete, unmodified, standard Apache License 2.0 text (verified by reading both the
header and the closing "APPENDIX: How to apply" section). It matches every package's
`<license>Apache-2.0</license>` tag (`farm_world`, `mode_arbitration`, `nav2_config`,
`robot_bringup`, `robot_description`, `teleop_node` — all six checked) and the README's
own "License: Apache-2.0" line. The trailing `Copyright [yyyy] [name of copyright
owner]` placeholder is the license's own standard template text (this is how GitHub's
own "Add a license" generator produces this exact file) — leaving it unfilled is normal
practice and not a defect.

---

## Summary

**Critical:** none remaining. The one critical finding from the previous audit
(`nav2_config`'s fresh-clone build failure) is resolved and was re-verified live in this
pass, not just assumed from the commit log.

**Functional (real behaviour gaps, not build-breaking):**
1. Neither costmap reacts to live `/scan` data — `global_costmap`'s `obstacle_layer` is
   defined but not wired into `plugins:`, and `local_costmap` has no obstacle layer at
   all (§2). Unchanged from previous audit.
2. `map_file`'s default and `run_trials.sh`'s hardcoded paths are both machine-specific
   absolute paths outside the repo — work today, not portable to a different clone
   location or user (§4).

**Minor (cosmetic / documentation drift):**
3. Two package.xml `<description>` fields still say "AMCL" / "Twist" where the code has
   moved on (`nav2_config`, `teleop_node`) — never touched by the dependency-fix commit,
   which only added/corrected `<exec_depend>` entries (§2).
4. `nav2_config` still depends on `nav2_amcl`, which nothing launches (§2, §5).
5. Dead YAML config (`amcl:`, `lifecycle_manager:`/`lifecycle_manager_slam:` blocks) —
   left in place on instruction, still dead, not a regression (§2).
6. One leftover "AMCL" mention in `full_system.launch.py`'s Quick-start docstring
   (line 17), outside the scope of either AMCL-wording fix round (§2).
7. `scout_mission.py:422` — `#nav.lifecycleShutdown()` commented out rather than
   removed or restored, reason undocumented (§2).
8. Three scripts (`generate_farm_map.py`, `analyse_trials.py`, `run_trials.sh`) have no
   `CMakeLists.txt` install rule / `ros2 run` entry point; all three remain directly
   runnable, so this is an inconsistency rather than a defect (§6).
9. README doesn't document `run_trials.sh` or `analyse_trials.py`, and understates
   `scout_mission.py`'s actual CSV-path behaviour (only covers the bare-name case,
   not the absolute-path case `run_trials.sh` actually uses) (§1, §9).

**Confirmed still working, re-verified rather than assumed:**
- Fresh-clone build of `nav2_config` (§R1, §6).
- All field-geometry constants across `farm.sdf`, `slam_coverage_drive.py`,
  `scout_mission.py`, and `generate_farm_map.py` (§4).
- The full `trial_results/` (60 files) → `analyse_trials.py` → `analysis_output/`
  pipeline, byte-for-byte (§8, §9).
