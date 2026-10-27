cmake_minimum_required(VERSION 3.16)
project(kuka_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config behavior_trees launch maps DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/wall_approach.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
