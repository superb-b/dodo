cmake_minimum_required(VERSION 3.8)
project(dodo_canbus)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(canbus_node
  src/multi_motor_control_node.cpp
  src/multi_motor_control_main.cpp
)

target_include_directories(canbus_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(canbus_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  canbus_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/test.py
  scripts/trajectory_player_node.py
  scripts/trajectory_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()