cmake_minimum_required(VERSION 3.8)
project(argus_odom_throttle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_throttle src/odom_throttle_node.cpp)
ament_target_dependencies(odom_throttle rclcpp nav_msgs)

install(TARGETS odom_throttle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
