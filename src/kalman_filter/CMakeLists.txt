cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "gcc")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(kalman_filter_node src/kalman_filter_node.cpp)
ament_target_dependencies(kalman_filter_node rclcpp sensor_msgs std_msgs Eigen3)

install(TARGETS
  kalman_filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

