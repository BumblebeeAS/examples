cmake_minimum_required(VERSION 3.12.2)
project(bluerov_tasks)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME})

# Add the worlds dir to GZ_SIM_RESOURCE_PATH so `world_name:=robosub2025`
# resolves robosub2025.world; its <include> URIs pull the models from Fuel.
ament_environment_hooks("hooks/bluerov_tasks_worlds.dsv.in")

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/actuators_stub.py
  scripts/arm_and_set_mode.py
  scripts/bluerov_bin_mission_tree.py
  scripts/bluerov_movement.py
  scripts/bluerov_square_mission_tree.py
  scripts/bluerov_torpedo_mission_tree.py
  scripts/goto.py
  scripts/locomotion_action_server.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
