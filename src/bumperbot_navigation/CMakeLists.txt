cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
)

add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(
  path_publisher
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
