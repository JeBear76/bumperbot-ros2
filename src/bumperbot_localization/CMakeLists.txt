cmake_minimum_required(VERSION 3.8)
project(bumperbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/kalman_filter.py
  ${PROJECT_NAME}/imu_republisher.py
  ${PROJECT_NAME}/odometry_motion_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
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

add_executable(kalman_filter src/kalman_filter.cpp)
ament_target_dependencies(
  kalman_filter
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(
  imu_republisher
  rclcpp
  sensor_msgs
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS 
    path_publisher
    kalman_filter
    imu_republisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
