cmake_minimum_required(VERSION 3.8)
project(artificial_occupy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(src ${CMAKE_CURRENT_SOURCE_DIR}/src/)
add_executable(artificial_occupy ${src}/main.cpp)
ament_target_dependencies(artificial_occupy 
  rclcpp 
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  std_srvs
  nav_msgs
  tf2_geometry_msgs
  geometry_msgs)

install(TARGETS 
  artificial_occupy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
