cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scout_mission.py
  RENAME scout_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/slam_coverage_drive.py
  RENAME slam_coverage_drive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
