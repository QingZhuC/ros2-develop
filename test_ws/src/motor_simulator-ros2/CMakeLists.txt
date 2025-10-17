cmake_minimum_required(VERSION 3.8)
project(motor_simulator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include/)

add_executable(motor_simulator_node src/motor_simulator_node.cpp)

ament_target_dependencies(motor_simulator_node rclcpp std_msgs geometry_msgs)

add_executable(pid_control src/pid_control.cpp)
ament_target_dependencies(pid_control rclcpp std_msgs)


install(TARGETS
  motor_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  pid_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()