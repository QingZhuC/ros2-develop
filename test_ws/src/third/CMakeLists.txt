cmake_minimum_required(VERSION 3.8)
project(third)

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

add_executable(signal_generate_third src/signal_generate_third.cpp)
ament_target_dependencies(signal_generate_third rclcpp std_msgs)

install(TARGETS
  signal_generate_third
  DESTINATION lib/${PROJECT_NAME})

add_executable(filter_node src/filter_node.cpp)
ament_target_dependencies(filter_node rclcpp std_msgs)

install(TARGETS
  filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
