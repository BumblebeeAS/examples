cmake_minimum_required(VERSION 3.12.2)
project(multivehicle_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(px4_offboard_demo src/PX4OffboardDemo.cpp)
ament_target_dependencies(px4_offboard_demo rclcpp geometry_msgs px4_msgs Eigen3)
target_compile_features(px4_offboard_demo PUBLIC c_std_99 cxx_std_17)
install(TARGETS px4_offboard_demo DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/bluerov_movement.py
  scripts/blueboat_thrust_mixer.py
  scripts/blueboat_waypoint_controller.py
  scripts/blueboat_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
