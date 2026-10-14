cmake_minimum_required(VERSION 3.8)
project(argus_usb_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(usb_cam_node
  src/usb_cam_node.cpp
)
target_include_directories(usb_cam_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(usb_cam_node ${OpenCV_LIBS})
ament_target_dependencies(usb_cam_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

install(TARGETS usb_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
