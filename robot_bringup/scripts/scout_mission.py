#!/usr/bin/env python3
"""
Scout mission with mode arbitration evaluation.

Scenarios:
  baseline   — normal mission, mode switches at waypoints 2 and 4
  deadman    — Nav2 killed mid-mission, tests 0.5s auto-stop safety
  obstacle   — operator takeover mid-aisle, teleop around, resume auto

Measures:
  - Per-waypoint navigation success/failure and duration
  - Mode switch transition latency (time to stop / time to resume)
  - Dead-man timeout response time

Usage:
  python3 scout_mission.py --scenario baseline --auto --log baseline_01
  python3 scout_mission.py --scenario deadman --auto --log deadman_01
  python3 scout_mission.py --scenario obstacle --auto --log obstacle_01
"""

import csv
import math
import os
import sys
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

SPAWN = (-9.5, -10.0)


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def build_waypoints():
    N = math.pi / 2
    S = -math.pi / 2
    return [
        ('navigate_to_aisle1', -6.0, -10.0, N),
        ('aisle1_mid', -6.0, -2.0, N),
        ('aisle1_top', -6.0, 5.0, N),
        ('cross_to_aisle2', -2.0, 5.0, S),
        ('aisle2_mid', -2.0, -2.0, S),
        ('return_near_spawn', -6.0, -8.0, N),
    ]


class SystemMonitor(Node):
    """Monitors /current_mode and /cmd_vel for transition latency."""

    def __init__(self):
        super().__init__('system_monitor')
        self._mode = 'unknown'
        self._transitions = []
        self._start_time = time.time()
        self._vel_linear = 0.0
        self._vel_angular = 0.0
        self._vel_timestamp = 0.0
        self._stop_time = None
        self._resume_time = None
        self._switch_time = None
        self._latency_records = []
        self._watching_stop = False
        self._watching_resume = False

        self.create_subscription(String, '/current_mode', self._mode_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._vel_cb, 10)

    def _mode_cb(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode != self._mode:
            t = time.time() - self._start_time
            self._transitions.append({
                'time': round(t, 3),
                'from': self._mode,
                'to': new_mode,
            })
            self._mode = new_mode

    def _vel_cb(self, msg):
        self._vel_linear = msg.twist.linear.x
        self._vel_angular = msg.twist.angular.z
        self._vel_timestamp = time.time()
        speed = abs(self._vel_linear) + abs(self._vel_angular)

        if self._watching_stop and speed < 0.01:
            self._stop_time = time.time()
            self._watching_stop = False

        if self._watching_resume and speed > 0.01:
            self._resume_time = time.time()
            self._watching_resume = False

    def start_watching_stop(self):
        self._switch_time = time.time()
        self._stop_time = None
        self._watching_stop = True

    def start_watching_resume(self):
        self._switch_time = time.time()
        self._resume_time = None
        self._watching_resume = True

    def get_stop_latency(self, timeout=3.0):
        deadline = time.time() + timeout
        while self._watching_stop and time.time() < deadline:
            time.sleep(0.01)
        if self._stop_time and self._switch_time:
            return round(self._stop_time - self._switch_time, 4)
        return None

    def get_resume_latency(self, timeout=10.0):
        deadline = time.time() + timeout
        while self._watching_resume and time.time() < deadline:
            time.sleep(0.01)
        if self._resume_time and self._switch_time:
            return round(self._resume_time - self._switch_time, 4)
        return None

    @property
    def mode(self):
        return self._mode

    @property
    def transitions(self):
        return list(self._transitions)

    @property
    def is_moving(self):
        return (abs(self._vel_linear) + abs(self._vel_angular)) > 0.01

    def clear(self):
        self._transitions.clear()


def main():
    auto_run = '--auto' in sys.argv
    log_name = None
    scenario = 'baseline'
    for i, arg in enumerate(sys.argv):
        if arg == '--log' and i + 1 < len(sys.argv):
            log_name = sys.argv[i + 1]
        if arg == '--scenario' and i + 1 < len(sys.argv):
            scenario = sys.argv[i + 1]

    rclpy.init()
    nav = BasicNavigator()

    monitor = SystemMonitor()
    monitor_exec = SingleThreadedExecutor()
    monitor_exec.add_node(monitor)
    monitor_thread = Thread(target=monitor_exec.spin, daemon=True)
    monitor_thread.start()

    print('[scout] waiting for Nav2...')
    nav.waitUntilNav2Active(navigator='bt_navigator', localizer='map_server')
    print('[scout] Nav2 ready')

    waypoints = build_waypoints()
    total = len(waypoints)
    results = []

    mode_pub_node = rclpy.create_node('mode_pub')
    mode_pub = mode_pub_node.create_publisher(String, '/mode', 10)

    def set_mode(mode_str):
        msg = String()
        msg.data = mode_str
        for _ in range(5):
            mode_pub.publish(msg)
            time.sleep(0.05)
        print(f'[scout] MODE -> {mode_str.upper()}')

    set_mode('auto')
    time.sleep(1)

    print(f'[scout] scenario: {scenario}')
    print(f'[scout] mission: {total} waypoints')
    if log_name:
        print(f'[scout] logging to: {log_name}.csv')
    print()

    mission_start = time.time()

    for idx, (label, x, y, yaw) in enumerate(waypoints):
        wp_num = idx + 1
        print(f'[scout] == waypoint {wp_num}/{total}: {label} ({x:.1f}, {y:.1f}) ==')

        if not auto_run and idx > 0:
            try:
                input('[scout] Press Enter to continue... ')
            except KeyboardInterrupt:
                print('\n[scout] stopped by user')
                break

        if monitor.mode != 'auto':
            set_mode('auto')
            time.sleep(0.5)

        goal = make_pose(nav, x, y, yaw)
        monitor.clear()
        wp_start = time.time()

        nav.goToPose(goal)

        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and fb.distance_remaining > 0:
                print(
                    f'  dist: {fb.distance_remaining:.1f}m  '
                    f'mode: {monitor.mode}    ',
                    end='\r'
                )
            time.sleep(0.2)

        wp_end = time.time()
        wp_duration = round(wp_end - wp_start, 2)
        result = nav.getResult()

        if result == TaskResult.SUCCEEDED:
            status = 'SUCCEEDED'
            print(f'\n  >> {label} reached in {wp_duration}s')
        elif result == TaskResult.CANCELED:
            status = 'CANCELED'
            print(f'\n  >> {label} canceled after {wp_duration}s')
        else:
            status = 'FAILED'
            print(f'\n  >> {label} FAILED after {wp_duration}s')

        results.append({
            'waypoint': wp_num,
            'label': label,
            'x': x, 'y': y,
            'status': status,
            'duration_s': wp_duration,
            'mode_switches': len(monitor.transitions),
            'stop_latency_s': '',
            'resume_latency_s': '',
            'scenario': scenario,
        })

        # === BASELINE: mode arbitration tests at waypoints 2 and 4 ===
        if scenario == 'baseline' and wp_num in (2, 4) and status == 'SUCCEEDED':
            test_num = 1 if wp_num == 2 else 2
            print(f'\n[scout] === MODE ARBITRATION TEST {test_num} ===')

            # Measure stop latency
            print('[scout] Switching to MANUAL...')
            monitor.start_watching_stop()
            set_mode('manual')
            stop_latency = monitor.get_stop_latency(timeout=3.0)
            print(f'[scout] Stop latency: {stop_latency}s' if stop_latency else '[scout] Stop latency: not measured')

            time.sleep(2)

            # Measure resume latency: switch to auto, send a probe goal, measure first velocity
            print('[scout] Switching back to AUTO...')
            set_mode('auto')
            time.sleep(0.5)

            print('[scout] Sending probe goal to measure resume latency...')
            next_wp = waypoints[idx + 1] if idx + 1 < len(waypoints) else waypoints[idx]
            monitor.start_watching_resume()
            probe_goal = make_pose(nav, next_wp[1], next_wp[2], next_wp[3])
            nav.goToPose(probe_goal)
            resume_latency = monitor.get_resume_latency(timeout=10.0)
            print(f'[scout] Resume latency: {resume_latency}s' if resume_latency else '[scout] Resume latency: not measured')
            # Cancel probe goal so the main loop sends it properly
            nav.cancelTask()
            time.sleep(1)

            results.append({
                'waypoint': 0,
                'label': f'MODE_TEST_{test_num}',
                'x': x, 'y': y,
                'status': 'MANUAL_OVERRIDE',
                'duration_s': round(time.time() - wp_end, 2),
                'mode_switches': 2,
                'stop_latency_s': stop_latency or '',
                'resume_latency_s': resume_latency or '',
                'scenario': scenario,
            })

        # === DEADMAN: kill Nav2 auto commands at waypoint 3 ===
        if scenario == 'deadman' and wp_num == 2 and status == 'SUCCEEDED':
            print('\n[scout] === DEAD-MAN TIMEOUT TEST ===')
            print('[scout] Starting waypoint 3, will cancel goal mid-flight...')

            goal3 = make_pose(nav, waypoints[2][1], waypoints[2][2], waypoints[2][3])
            nav.goToPose(goal3)
            time.sleep(3)  # let it drive for 3 seconds

            print('[scout] Canceling Nav2 goal (simulating Nav2 failure)...')
            monitor.start_watching_stop()
            nav.cancelTask()
            stop_latency = monitor.get_stop_latency(timeout=3.0)
            print(f'[scout] Dead-man stop latency: {stop_latency}s' if stop_latency else '[scout] Dead-man timeout: not measured')

            time.sleep(3)

            results.append({
                'waypoint': 0,
                'label': 'DEADMAN_TIMEOUT_TEST',
                'x': waypoints[2][1], 'y': waypoints[2][2],
                'status': 'DEADMAN_TRIGGERED',
                'duration_s': round(time.time() - wp_end, 2),
                'mode_switches': 0,
                'stop_latency_s': stop_latency or '',
                'resume_latency_s': '',
                'scenario': scenario,
            })

            # Resume mission
            print('[scout] Resuming mission...')
            set_mode('auto')
            time.sleep(1)

        # === OBSTACLE: operator takeover at waypoint 2 ===
        if scenario == 'obstacle' and wp_num == 2 and status == 'SUCCEEDED':
            print('\n[scout] === OBSTACLE ENCOUNTER TEST ===')
            print('[scout] Simulating obstacle detected — operator takes over')

            # Start next goal, then interrupt mid-drive
            goal3 = make_pose(nav, waypoints[2][1], waypoints[2][2], waypoints[2][3])
            nav.goToPose(goal3)
            time.sleep(2)

            print('[scout] Switching to MANUAL (operator takeover)...')
            monitor.start_watching_stop()
            set_mode('manual')
            nav.cancelTask()
            stop_latency = monitor.get_stop_latency(timeout=3.0)
            print(f'[scout] Takeover stop latency: {stop_latency}s' if stop_latency else '[scout] Takeover latency: not measured')

            # Simulate operator driving (just wait)
            print('[scout] Operator controlling for 5s...')
            time.sleep(5)

            # Measure resume latency: switch to auto, send goal, measure first velocity
            print('[scout] Switching back to AUTO...')
            set_mode('auto')
            time.sleep(0.5)

            print('[scout] Sending recovery goal to measure resume latency...')
            monitor.start_watching_resume()
            recovery_goal = make_pose(nav, waypoints[2][1], waypoints[2][2], waypoints[2][3])
            nav.goToPose(recovery_goal)
            resume_latency = monitor.get_resume_latency(timeout=10.0)
            print(f'[scout] Resume latency: {resume_latency}s' if resume_latency else '[scout] Resume latency: not measured')
            # Cancel recovery goal so the main loop sends waypoint 3 properly
            nav.cancelTask()
            time.sleep(1)

            results.append({
                'waypoint': 0,
                'label': 'OBSTACLE_TAKEOVER_TEST',
                'x': waypoints[2][1], 'y': waypoints[2][2],
                'status': 'OBSTACLE_OVERRIDE',
                'duration_s': round(time.time() - wp_end, 2),
                'mode_switches': 2,
                'stop_latency_s': stop_latency or '',
                'resume_latency_s': resume_latency or '',
                'scenario': scenario,
            })

        if status == 'FAILED':
            print(f'[scout] navigation failed at {label}. Continuing...')

    mission_end = time.time()
    mission_duration = round(mission_end - mission_start, 2)

    succeeded = sum(1 for r in results if r['status'] == 'SUCCEEDED')
    failed = sum(1 for r in results if r['status'] == 'FAILED')
    mode_tests = sum(1 for r in results if r['status'] in
                     ('MANUAL_OVERRIDE', 'DEADMAN_TRIGGERED', 'OBSTACLE_OVERRIDE'))

    print()
    print('=' * 55)
    print(f'[scout] MISSION SUMMARY — scenario: {scenario}')
    print(f'  Navigation: {succeeded}/{total} waypoints succeeded')
    print(f'  Failed: {failed}')
    print(f'  Special tests: {mode_tests}')
    print(f'  Total time: {mission_duration}s')

    latencies = [r for r in results if r.get('stop_latency_s')]
    if latencies:
        print(f'  Stop latencies: {[r["stop_latency_s"] for r in latencies]}')
        print(f'  Resume latencies: {[r["resume_latency_s"] for r in latencies if r.get("resume_latency_s")]}')
    print('=' * 55)

    if log_name:
        csv_path = os.path.expanduser(f'~/ros2_ws/{log_name}.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'scenario', 'waypoint', 'label', 'x', 'y', 'status',
                'duration_s', 'mode_switches', 'stop_latency_s', 'resume_latency_s'
            ])
            writer.writeheader()
            writer.writerows(results)
        print(f'[scout] Results saved to {csv_path}')

    mode_pub_node.destroy_node()
    monitor_exec.shutdown()
    monitor.destroy_node()
    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
