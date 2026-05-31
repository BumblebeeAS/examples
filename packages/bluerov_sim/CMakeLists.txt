cmake_minimum_required(VERSION 3.12.2)
project(bluerov_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config models urdf
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/world_objects_to_markers.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(hooks/setup.dsv)

ament_package()
