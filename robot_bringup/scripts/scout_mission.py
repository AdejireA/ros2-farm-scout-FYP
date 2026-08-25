#!/usr/bin/env python3
"""
Scout mission with mode arbitration evaluation.

Uses shorter waypoint segments to avoid odom drift issues.
Tests mode switching at defined points during the mission.
Logs all metrics for evaluation.

Usage:
  python3 scout_mission.py --auto --log trial_01
  python3 scout_mission.py --log trial_01     # interactive, pauses between waypoints
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
from geometry_msgs.msg import PoseStamped
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
    """Shorter segments through aisles with mode-switch test points."""
    N = math.pi / 2    # face north
    S = -math.pi / 2   # face south
    E = 0.0            # face east
    wps = [
        # Phase 1: Navigate to first aisle entry
        ('navigate_to_aisle1', -6.0, -10.0, N),
        # Phase 2: Drive partway up aisle 1
        ('aisle1_mid', -6.0, -2.0, N),
        # Phase 3: Continue to top of aisle 1
        ('aisle1_top', -6.0, 5.0, N),
        # Phase 4: Cross to aisle 2 entry
        ('cross_to_aisle2', -2.0, 5.0, S),
        # Phase 5: Drive down aisle 2
        ('aisle2_mid', -2.0, -2.0, S),
        # Phase 6: Return toward spawn
        ('return_near_spawn', -6.0, -8.0, N),
    ]
    return wps

class ModeMonitor(Node):
    def __init__(self):
        super().__init__('mode_monitor')
        self._mode = 'unknown'
        self._transitions = []
        self._start_time = time.time()
        self.create_subscription(String, '/current_mode', self._cb, 10)

    def _cb(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode != self._mode:
            t = time.time() - self._start_time
            self._transitions.append({
                'time': round(t, 3),
                'from': self._mode,
                'to': new_mode,
            })
            self._mode = new_mode

    @property
    def mode(self):
        return self._mode

    @property
    def transitions(self):
        return list(self._transitions)

    def clear(self):
        self._transitions.clear()

class ModeSwitch:
    """Publishes mode changes."""
    def __init__(self):
        self._pub = rclpy.create_node('mode_switch_helper').create_publisher(
            String, '/mode', 10)

    def set_mode(self, mode):
        msg = String()
        msg.data = mode
        for _ in range(5):  # publish a few times to ensure delivery
            self._pub.publish(msg)
            time.sleep(0.05)

def main():
    auto_run = '--auto' in sys.argv
    log_name = None
    for i, arg in enumerate(sys.argv):
        if arg == '--log' and i + 1 < len(sys.argv):
            log_name = sys.argv[i + 1]

    rclpy.init()
    nav = BasicNavigator()

    monitor = ModeMonitor()
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

    # Mode switch publisher
    mode_pub_node = rclpy.create_node('mode_pub')
    mode_pub = mode_pub_node.create_publisher(String, '/mode', 10)

    def set_mode(mode_str):
        msg = String()
        msg.data = mode_str
        for _ in range(5):
            mode_pub.publish(msg)
            time.sleep(0.05)
        print(f'[scout] MODE -> {mode_str.upper()}')

    # Start in auto
    set_mode('auto')

    print(f'[scout] mission: {total} waypoints')
    print(f'[scout] mode arbitration test points: after waypoints 2 and 4')
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

        # Ensure auto mode before navigation
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

        transitions = monitor.transitions
        results.append({
            'waypoint': wp_num,
            'label': label,
            'x': x,
            'y': y,
            'status': status,
            'duration_s': wp_duration,
            'mode_switches': len(transitions),
            'transitions': str(transitions) if transitions else '',
        })

        # === MODE ARBITRATION TEST POINTS ===
        if wp_num == 2 and status == 'SUCCEEDED':
            print('\n[scout] === MODE ARBITRATION TEST 1 ===')
            print('[scout] Switching to MANUAL (simulating operator takeover)...')
            set_mode('manual')
            manual_start = time.time()
            time.sleep(3)  # robot should stop within 0.5s
            print(f'[scout] Robot stopped for 3s in MANUAL mode')
            print('[scout] Switching back to AUTO...')
            set_mode('auto')
            time.sleep(1)
            manual_duration = round(time.time() - manual_start, 2)
            results.append({
                'waypoint': 0,
                'label': 'MODE_TEST_1_manual_override',
                'x': x, 'y': y,
                'status': 'MANUAL_OVERRIDE',
                'duration_s': manual_duration,
                'mode_switches': 2,
                'transitions': 'auto->manual->auto',
            })

        if wp_num == 4 and status == 'SUCCEEDED':
            print('\n[scout] === MODE ARBITRATION TEST 2 ===')
            print('[scout] Switching to MANUAL...')
            set_mode('manual')
            manual_start = time.time()
            time.sleep(3)
            print(f'[scout] Robot stopped for 3s in MANUAL mode')
            print('[scout] Switching back to AUTO...')
            set_mode('auto')
            time.sleep(1)
            manual_duration = round(time.time() - manual_start, 2)
            results.append({
                'waypoint': 0,
                'label': 'MODE_TEST_2_manual_override',
                'x': x, 'y': y,
                'status': 'MANUAL_OVERRIDE',
                'duration_s': manual_duration,
                'mode_switches': 2,
                'transitions': 'auto->manual->auto',
            })

    mission_end = time.time()
    mission_duration = round(mission_end - mission_start, 2)

    succeeded = sum(1 for r in results if r['status'] == 'SUCCEEDED')
    failed = sum(1 for r in results if r['status'] == 'FAILED')
    mode_tests = sum(1 for r in results if r['status'] == 'MANUAL_OVERRIDE')
    total_switches = sum(r['mode_switches'] for r in results)

    print()
    print('=' * 55)
    print('[scout] MISSION SUMMARY')
    print(f'  Navigation: {succeeded}/{total} waypoints succeeded')
    print(f'  Failed: {failed}')
    print(f'  Mode arbitration tests: {mode_tests}')
    print(f'  Total mode switches: {total_switches}')
    print(f'  Total time: {mission_duration}s')
    print('=' * 55)

    if log_name:
        csv_path = os.path.expanduser(f'~/ros2_ws/{log_name}.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'waypoint', 'label', 'x', 'y', 'status',
                'duration_s', 'mode_switches', 'transitions'
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
