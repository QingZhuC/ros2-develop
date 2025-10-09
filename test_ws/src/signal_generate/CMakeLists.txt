cmake_minimum_required(VERSION 3.8)
project(signal_generate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(signal_generate src/signal_generate.cpp)
ament_target_dependencies(signal_generate rclcpp std_msgs)

install(TARGETS
  signal_generate
  DESTINATION lib/${PROJECT_NAME})

add_executable(signal_processing src/signal_processing.cpp)
ament_target_dependencies(signal_processing rclcpp std_msgs)

install(TARGETS
  signal_generate
  signal_processing
  DESTINATION lib/${PROJECT_NAME})

ament_package()
