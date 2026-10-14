cmake_minimum_required(VERSION 3.8)
project(argus_bmi270)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(BMI270_API_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/BMI270_SensorAPI)

add_library(bmi270_api STATIC
  ${BMI270_API_DIR}/bmi2.c
  ${BMI270_API_DIR}/bmi270.c
)
target_include_directories(bmi270_api PUBLIC ${BMI270_API_DIR})
set_target_properties(bmi270_api PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)

add_executable(bmi270_fast_node
  src/bmi270_fast_node.cpp
  src/bmi2_glue.cpp
)
target_include_directories(bmi270_fast_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${BMI270_API_DIR}
)
target_link_libraries(bmi270_fast_node bmi270_api)
ament_target_dependencies(bmi270_fast_node
  rclcpp
  sensor_msgs
)

install(TARGETS bmi270_fast_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
