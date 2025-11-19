cmake_minimum_required(VERSION 3.8)
project(tortoisebot_localization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/multiwaypoint.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME multiwaypoint
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

