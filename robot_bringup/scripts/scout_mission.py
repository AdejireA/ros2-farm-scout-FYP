#!/usr/bin/env python3
"""
Scout mission — boustrophedon traversal of the 4 inter-row aisles.

The robot visits each aisle in sequence, alternating direction (south→north,
north→south, ...) so it never doubles back across the field. After the last
aisle it returns to the spawn point.

Aisle x positions: -6, -2, 2, 6 — gap centerlines between crop rows (rows are
at -8, -4, 0, 4, 8), not the row positions themselves. Each centerline sits
2m from its two flanking rows, well within LIDAR range, so both rows stay
visible without the robot driving through either one's collision geometry.
Y travel range: -10 to +10  (1 m clear of outermost plants at y = ±9)

Usage (Nav2 must be active and robot localised first):
  ros2 run robot_bringup scout_mission
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Gap centerlines between crop rows (2m from each flanking row, within LIDAR
# range) — NOT the row x-positions themselves (those are -8,-4,0,4,8).
AISLES   = [-6.0, -2.0, 2.0, 6.0]
Y_SOUTH  = -10.0
Y_NORTH  =  10.0
SPAWN    = (-9.5, -10.0)


def pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def build_waypoints(nav: BasicNavigator) -> list:
    wps = []
    for i, x in enumerate(AISLES):
        if i % 2 == 0:
            wps.append(pose(nav, x, Y_SOUTH,  math.pi / 2))   # face north
            wps.append(pose(nav, x, Y_NORTH,  math.pi / 2))
        else:
            wps.append(pose(nav, x, Y_NORTH, -math.pi / 2))   # face south
            wps.append(pose(nav, x, Y_SOUTH, -math.pi / 2))
    wps.append(pose(nav, SPAWN[0], SPAWN[1], math.pi / 2))    # return to spawn
    return wps


def main():
    rclpy.init()
    nav = BasicNavigator()

    print('[scout] waiting for Nav2...')
    nav.waitUntilNav2Active()

    waypoints = build_waypoints(nav)
    total = len(waypoints)
    print(f'[scout] starting mission — {total} waypoints')

    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            idx = fb.current_waypoint
            wp = waypoints[idx]
            print(
                f'[scout] waypoint {idx + 1}/{total}  '
                f'x={wp.pose.position.x:+.0f}  '
                f'y={wp.pose.position.y:+.0f}'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[scout] mission complete — all aisles traversed')
    elif result == TaskResult.CANCELED:
        print('[scout] mission cancelled')
    else:
        print('[scout] mission failed')

    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
