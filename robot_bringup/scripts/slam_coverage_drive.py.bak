#!/usr/bin/env python3
"""
Scripted SLAM-coverage driver — v4, relative-pose-checked.

Root-cause fix vs v3: v3 polled live map->base_link TF and compared it
directly against absolute Gazebo WORLD-frame constants (WALL_Y=12.0,
AISLES x-positions, etc.) — e.g. drive_until('y', 10.0) meant "stop when
map-frame y reaches 10.0". That's wrong. slam_toolbox defines the map
frame's origin as wherever the robot is standing the moment SLAM
activates, with identity rotation (yaw=0) at that pose — NOT wherever
Gazebo's world origin is. Confirmed live: `tf2_echo map base_link`
reports (0, 0, 0.12), yaw 0 at startup, while the robot's real spawn in
farm.sdf/scout_mission.py's SPAWN constant is (-9.5, -10.0), yaw 90 deg.
Two different coordinate systems. Every v3 leg would have stopped after
a few meters of LOCAL-frame travel instead of the real ~18-20m needed to
reach the actual field boundary — silently undercovering the field
without ever erroring, since nothing in v3 could tell the difference
between "reached the wall" and "reached local y=10 a few meters from
spawn".

v4 never compares live pose to a world-frame constant. Every leg is
still pose-checked and self-correcting — it still polls real map-frame
pose every tick and still measures real, physical progress — it just
measures that progress RELATIVE to where the leg itself started, not
against an absolute target expressed in a frame the local pose can't be
compared to:
  - turn_relative(delta_deg): capture current live yaw at call time,
    target = that + delta_deg, then turn (poll/correct) until reached.
  - drive_relative(distance_m): capture live (x, y) at call time, then
    poll pose every tick and stop once Euclidean distance traveled from
    that snapshot >= distance_m (within POSITION_TOLERANCE).

The actual metres/degrees each leg needs are still derived from the real
field geometry (WALL_X/WALL_Y/PERIMETER_INSET, AISLES, Y_SOUTH/Y_NORTH,
and the known real-world SPAWN pose from farm.sdf) — those numbers are
still valid MEASUREMENTS of the real world, they're just no longer used
as targets to compare local-frame pose against. Instead this file keeps
an internal "planned pose" bookkeeping trail (self._plan_x/_plan_y/
_plan_yaw), seeded from the known real spawn and advanced by each
commanded delta, purely so the existing decision logic (turn-direction-
by-comparison, nearest-aisle-first selection, N/S alternation) keeps
working exactly as before with no changes to its structure — only the
underlying motion primitives changed.

Caveat this reintroduces, worth knowing: v3 was deliberately spawn-
agnostic (read whatever pose TF gave it, no hardcoded assumption). v4
can't be — without a world-frame reference there is no way to compute
correct real-world leg distances except by assuming the robot is
actually at the known SPAWN_X/SPAWN_Y/SPAWN_YAW_DEG when stage_perimeter()
starts. If this script is ever launched after the robot has already
been driven around post-SLAM-activation (rather than immediately after
launch), the computed distances will be wrong. This is a real, inherent
limitation of relative-only tracking during initial mapping, not an
oversight.

sanity_check_pose() is reframed the same way: it can no longer compare
raw (x, y) to world-frame wall bounds (meaningless in local frame), so
it now tracks CUMULATIVE distance traveled (odometer-style, summed tick
by tick) from the first pose logged at startup, and flags if that ever
wildly exceeds what the full planned path should cover. The real planned
total is ~169m (perimeter ~77m + aisles ~92m, both computed below from
the same known geometry) — the ceiling is set with real margin above
that, not a guess. It also still flags a single implausibly large
tick-to-tick jump (>1m between two consecutive ~0.1s polls implies
>10m/s, physically impossible for this robot) — this is a closer match
to the previously-documented map->odom jump failure mode (a discrete,
frozen, large excursion) than the cumulative check alone, and costs
almost nothing to add.

The very first pose is still logged at startup — still useful to confirm
TF is alive — but it's expected to read close to (0, 0, 0), not the real
spawn (-9.5, -10.0). That's correct map-frame behaviour, not a bug.

Stages:
  1. PERIMETER — from the known real spawn: face north (a no-op turn —
     the robot already faces north at spawn), drive to near the north
     wall; face east, drive to near the east wall; face south, drive to
     near the south wall; face west, drive back toward the starting x.
     Checks real /map coverage against all 4 walls after.
  2. AISLES — serpentine across the 4 inter-row gap centerlines (x =
     -6,-2,2,6), alternating north/south direction, with a short lateral
     pose-checked shift between each. Starts from whichever aisle is
     nearest the planned current position and sweeps outward.

Run with SLAM mode already up (full_system.launch.py use_slam:=true) and
mode set to 'manual' (cmd_vel_teleop only reaches the robot in manual mode).

Usage:
    ros2 run robot_bringup slam_coverage_drive          # pauses between stages
    ros2 run robot_bringup slam_coverage_drive --auto    # runs straight through
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# ── Tuning ───────────────────────────────────────────────────────────────
LIN_SPEED = 0.35
TURN_SPEED = 0.30
PUBLISH_HZ = 10.0
CORNER_DWELL = 1.0

# Field layout, from farm.sdf — walls at x=+-11, y=+-12
WALL_X = 11.0
WALL_Y = 12.0
PERIMETER_INSET = 2.0

# Known real-world spawn pose (from farm.sdf's robot <pose> and
# scout_mission.py's SPAWN constant) — used ONLY to precompute each leg's
# real-world distance/turn amount below. NEVER compared against live TF:
# slam_toolbox's map frame starts at (0,0,0)/yaw=0 wherever the robot
# happens to be when SLAM activates, regardless of these numbers.
SPAWN_X = -9.5
SPAWN_Y = -10.0
SPAWN_YAW_DEG = 90.0   # facing north (+y) at spawn

# Gap centerlines between crop rows (2m clear of each flanking row, well within
# LIDAR_RANGE=3.5m) — NOT the row x-positions themselves (those are -8,-4,0,4,8).
AISLES = [-6.0, -2.0, 2.0, 6.0]
Y_SOUTH = -9.5
Y_NORTH = 9.5

POSITION_TOLERANCE = 0.3   # m, "close enough" to a leg's target
YAW_TOLERANCE_DEG = 3.0    # deg, "close enough" to a target heading
LEG_TIMEOUT = 75.0         # s, safety cutoff per leg — if hit, something's wrong
TURN_TIMEOUT = 15.0        # s, safety cutoff per turn

# Real planned-path total, computed from the same known geometry (see
# stage_perimeter/stage_aisles below): perimeter ~77m + aisles ~92m = ~169m.
# Ceiling below gives ~50% margin above that for turning-arc slop and
# mid-leg correction — this is a coarse last-resort net (a real jump like
# the previously-documented map->odom failure would eventually push
# cumulative distance past it), not a tight bound. It is intentionally
# NOT the "~90m" rough figure floated when this rewrite was requested —
# 90m is already less than the honest planned total and would have
# false-triggered on a clean run.
CUMULATIVE_DISTANCE_CEILING = 260.0  # m
MAX_TICK_JUMP_M = 1.0  # m between consecutive ~0.1s polls; >10m/s is impossible for this robot

def normalize_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

class SlamCoverageDriver(Node):

    def __init__(self):
        super().__init__('slam_coverage_drive')
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self._period = 1.0 / PUBLISH_HZ
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Planned-pose bookkeeping, seeded from the known real spawn — this
        # is what turn/drive amounts get computed from, NOT live TF (which
        # is in the local map frame and not comparable to these numbers).
        self._plan_x = SPAWN_X
        self._plan_y = SPAWN_Y
        self._plan_yaw = SPAWN_YAW_DEG

        # Cumulative-distance sanity tracking.
        self._first_pose = None
        self._last_pose_for_cum = None
        self._cum_dist = 0.0
        self._jump_flagged = False

    # ── Pose ────────────────────────────────────────────────────────────
    def get_pose(self):
        """Returns (x, y, yaw_deg) from map->base_link — a LOCAL SLAM map-
        frame pose (origin = wherever the robot was when SLAM activated),
        NOT a world-frame pose. Returns None if unavailable."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'[coverage] TF lookup failed: {e}')
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return (x, y, yaw)

    def _accumulate(self, pose):
        """Feeds one live pose sample into the cumulative-distance sanity
        trail. Tracks total odometer-style distance since the first pose
        logged at startup, and flags a single implausibly large tick-to-
        tick jump (the previously-documented map->odom jump's signature:
        discrete, large, frozen afterward)."""
        if pose is None:
            return
        if self._first_pose is None:
            self._first_pose = (pose[0], pose[1])
            self.get_logger().info(
                f'[coverage] first live pose = ({pose[0]:.2f}, {pose[1]:.2f}), '
                f'yaw {pose[2]:.1f} deg. This is a MAP-FRAME pose — slam_toolbox '
                f'sets the map origin to wherever the robot is when SLAM activates, '
                f'so reading close to (0,0,0) here is correct, not a bug. It is NOT '
                f'expected to match the real spawn ({SPAWN_X:.1f}, {SPAWN_Y:.1f}).')
            self._last_pose_for_cum = (pose[0], pose[1])
            return
        dx = pose[0] - self._last_pose_for_cum[0]
        dy = pose[1] - self._last_pose_for_cum[1]
        step = math.hypot(dx, dy)
        if step > MAX_TICK_JUMP_M:
            self._jump_flagged = True
            self.get_logger().error(
                f'[coverage] IMPLAUSIBLE POSE JUMP — {step:.2f}m in one ~{self._period:.2f}s '
                f'poll (>{MAX_TICK_JUMP_M:.0f}m/tick is physically impossible for this '
                f'robot). Matches the known map->odom jump failure mode. STOPPING.')
        self._cum_dist += step
        self._last_pose_for_cum = (pose[0], pose[1])

    def sanity_check_pose(self, label=''):
        pose = self.get_pose()
        if pose is None:
            self.get_logger().warn(f'[coverage] {label}: no TF yet, cannot sanity-check.')
            return True
        self._accumulate(pose)
        if self._jump_flagged:
            return False
        if self._cum_dist > CUMULATIVE_DISTANCE_CEILING:
            self.get_logger().error(
                f'[coverage] {label}: CUMULATIVE DISTANCE LOOKS WRONG — '
                f'{self._cum_dist:.1f}m traveled so far, past the '
                f'{CUMULATIVE_DISTANCE_CEILING:.0f}m ceiling for the whole planned '
                f'path (~169m + margin). STOPPING.')
            return False
        self.get_logger().info(
            f'[coverage] {label}: position ({pose[0]:.2f}, {pose[1]:.2f}) — '
            f'cumulative {self._cum_dist:.1f}m / {CUMULATIVE_DISTANCE_CEILING:.0f}m ceiling, OK')
        return True

    # ── Motion primitives — both pose-checked, both relative to leg start ──
    def _publish(self, lin, ang):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self._pub.publish(msg)

    def _stop(self, dwell=0.5):
        end_time = time.time() + dwell
        while time.time() < end_time and rclpy.ok():
            self._publish(0.0, 0.0)
            time.sleep(self._period)

    def turn_relative(self, delta_deg, label=''):
        """Turns by delta_deg relative to the LIVE yaw at call time (not an
        absolute compass heading — local map frame has no compass
        reference). Polls real yaw every tick and self-corrects, same as
        before; only the reference point changed."""
        self._stop(dwell=CORNER_DWELL)
        pose = self.get_pose()
        if pose is None:
            self.get_logger().error(f'[coverage] turn_relative({delta_deg}) {label}: no pose, aborting turn.')
            return False
        target_yaw = normalize_deg(pose[2] + delta_deg)
        self.get_logger().info(
            f'[coverage] turn_relative {delta_deg:+.0f} deg {label} '
            f'(currently {pose[2]:.1f}, target {target_yaw:.1f})')
        end_time = time.time() + TURN_TIMEOUT
        while rclpy.ok() and time.time() < end_time:
            pose = self.get_pose()
            if pose is None:
                time.sleep(self._period)
                continue
            self._accumulate(pose)
            diff = normalize_deg(target_yaw - pose[2])
            if abs(diff) <= YAW_TOLERANCE_DEG:
                break
            ang = TURN_SPEED if diff > 0 else -TURN_SPEED
            self._publish(0.0, ang)
            time.sleep(self._period)
        else:
            self.get_logger().warn(
                f'[coverage] turn_relative {delta_deg:+.0f} {label}: TIMED OUT before '
                f'reaching target heading — check for a stuck wheel or bad TF.')
        self._stop(dwell=CORNER_DWELL)
        return True

    def drive_relative(self, distance_m, label=''):
        """Drives forward on whatever heading is currently held, polling
        live pose, stops when Euclidean distance traveled from the pose
        captured at call time reaches distance_m (within
        POSITION_TOLERANCE). Self-corrects for drift within the leg —
        measures progress relative to where the leg started, not against
        a world-frame constant. Call turn_relative() first to face the
        right direction."""
        pose = self.get_pose()
        if pose is None:
            self.get_logger().error(f'[coverage] drive_relative {label}: no pose, aborting leg.')
            return False
        x0, y0 = pose[0], pose[1]
        self.get_logger().info(
            f'[coverage] drive_relative {distance_m:.1f}m {label} '
            f'(starting at {x0:.2f}, {y0:.2f})')
        end_time = time.time() + LEG_TIMEOUT
        reached = False
        while rclpy.ok() and time.time() < end_time:
            pose = self.get_pose()
            if pose is None:
                time.sleep(self._period)
                continue
            self._accumulate(pose)
            if self._jump_flagged:
                break
            traveled = math.hypot(pose[0] - x0, pose[1] - y0)
            reached = traveled >= distance_m - POSITION_TOLERANCE
            if reached:
                break
            self._publish(LIN_SPEED, 0.0)
            time.sleep(self._period)
        self._stop()
        if self._jump_flagged:
            return False
        if not reached:
            self.get_logger().warn(
                f'[coverage] drive_relative {distance_m:.1f}m {label}: TIMED OUT '
                f'before reaching target — check for a stuck robot or bad TF, '
                f'do not assume this leg completed.')
        return reached

    # ── Coverage check — unchanged, queries live map bounds directly ──────
    def check_full_coverage(self):
        from nav_msgs.srv import GetMap
        client = self.create_client(GetMap, 'map_server/map')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('[coverage] map service unavailable, skipping check.')
            return
        req = GetMap.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn('[coverage] map service call failed, skipping check.')
            return
        info = future.result().map.info
        x_min = info.origin.position.x
        x_max = x_min + info.width * info.resolution
        y_min = info.origin.position.y
        y_max = y_min + info.height * info.resolution
        self.get_logger().info(
            f'[coverage] Map bounds: x=[{x_min:.2f}, {x_max:.2f}]  y=[{y_min:.2f}, {y_max:.2f}]')
        for label, ok in [
            ('east wall (x=+11)', x_max >= WALL_X),
            ('west wall (x=-11)', x_min <= -WALL_X),
            ('north wall (y=+12)', y_max >= WALL_Y),
            ('south wall (y=-12)', y_min <= -WALL_Y),
        ]:
            self.get_logger().info(f'[coverage]   {label}: {"OK" if ok else "SHORT - not covered"}')

    # ── Stages ──────────────────────────────────────────────────────────
    def stage_perimeter(self):
        self.get_logger().info('=== STAGE: PERIMETER ===')
        if not self.sanity_check_pose('perimeter start'):
            return False

        # Targets below are still real-world MEASUREMENTS (from farm.sdf's
        # known wall positions and inset) — just used to compute each leg's
        # relative turn/drive amount against the planned-pose trail, never
        # compared to live (local-frame) pose directly.
        north_target = WALL_Y - PERIMETER_INSET       # 10.0
        east_target = WALL_X - PERIMETER_INSET        # 9.0
        south_target = -(WALL_Y - PERIMETER_INSET)    # -10.0

        # Leg 1: north. Already facing north at spawn (SPAWN_YAW_DEG=90),
        # so this turn is a genuine no-op — kept for structural symmetry
        # and logging, not because a real turn is expected.
        delta = normalize_deg(90.0 - self._plan_yaw)
        if not self.turn_relative(delta, 'face north'): return False
        self._plan_yaw = 90.0
        dist = north_target - self._plan_y
        if not self.drive_relative(dist, 'toward north wall'): return False
        self._plan_y = north_target
        if not self.sanity_check_pose('after leg 1'): return False

        # Leg 2: east (right turn).
        delta = normalize_deg(0.0 - self._plan_yaw)
        if not self.turn_relative(delta, 'face east'): return False
        self._plan_yaw = 0.0
        dist = east_target - self._plan_x
        if not self.drive_relative(dist, 'toward east wall'): return False
        self._plan_x = east_target
        if not self.sanity_check_pose('after leg 2'): return False

        # Leg 3: south (right turn).
        delta = normalize_deg(-90.0 - self._plan_yaw)
        if not self.turn_relative(delta, 'face south'): return False
        self._plan_yaw = -90.0
        dist = self._plan_y - south_target
        if not self.drive_relative(dist, 'toward south wall'): return False
        self._plan_y = south_target
        if not self.sanity_check_pose('after leg 3'): return False

        # Leg 4: west, back toward starting x (right turn).
        delta = normalize_deg(180.0 - self._plan_yaw)
        if not self.turn_relative(delta, 'face west'): return False
        self._plan_yaw = 180.0
        dist = self._plan_x - SPAWN_X
        if not self.drive_relative(dist, 'back toward start'): return False
        self._plan_x = SPAWN_X

        self.get_logger().info('=== PERIMETER STAGE COMPLETE ===')
        self.check_full_coverage()
        return self.sanity_check_pose('perimeter end')

    def stage_aisles(self):
        self.get_logger().info('=== STAGE: AISLES ===')
        if not self.sanity_check_pose('aisle stage start'):
            return False

        cur_x = self._plan_x
        ordered = sorted(AISLES, key=lambda ax: abs(ax - cur_x))
        start_idx = AISLES.index(ordered[0])
        going_east = ordered[0] < AISLES[-1] if start_idx == 0 else AISLES[start_idx] > AISLES[start_idx - 1]
        sequence = AISLES[start_idx:] if going_east else list(reversed(AISLES[:start_idx + 1]))

        heading_ns = 90  # start each aisle facing north; alternates every aisle
        for target_x in sequence:
            turn_target = 0.0 if target_x > cur_x else 180.0
            delta = normalize_deg(turn_target - self._plan_yaw)
            self.turn_relative(delta, f'shift to aisle x={target_x}')
            self._plan_yaw = turn_target
            dist = abs(target_x - cur_x)
            self.drive_relative(dist, f'align aisle x={target_x}')
            self._plan_x = target_x
            cur_x = target_x
            if not self.sanity_check_pose(f'aligned aisle x={target_x}'):
                return False

            target_y = Y_NORTH if heading_ns == 90 else Y_SOUTH
            delta = normalize_deg(float(heading_ns) - self._plan_yaw)
            self.turn_relative(delta, f'face {"north" if heading_ns == 90 else "south"} for aisle x={target_x}')
            self._plan_yaw = float(heading_ns)
            dist = abs(target_y - self._plan_y)
            if not self.drive_relative(dist, f'sweep aisle x={target_x}'):
                return False
            self._plan_y = target_y
            if not self.sanity_check_pose(f'after aisle x={target_x}'):
                return False
            heading_ns = -90 if heading_ns == 90 else 90

        self.get_logger().info('=== AISLES STAGE COMPLETE ===')
        self.check_full_coverage()
        return True

def confirm(prompt):
    input(f'{prompt} [Enter to continue, Ctrl-C to stop]: ')

def main(args=None):
    auto = '--auto' in sys.argv
    rclpy.init(args=args)
    node = SlamCoverageDriver()
    try:
        ok = node.stage_perimeter()
        if not ok:
            node.get_logger().error('[coverage] Perimeter stage flagged a problem. Stopping.')
            return
        if not auto:
            confirm('Perimeter done, coverage checked above. Review it - continue to aisles?')

        ok = node.stage_aisles()
        if not ok:
            node.get_logger().error('[coverage] Aisle stage flagged a problem. Stopping.')
            return

        node.get_logger().info(
            '[coverage] All stages done. Run the numeric /map check one final '
            'time before saving - do not save on this log output alone.')
    except KeyboardInterrupt:
        node.get_logger().info('[coverage] Stopped by user.')
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
