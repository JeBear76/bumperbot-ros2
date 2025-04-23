cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(turtlesim_kinematics src/turtlesim_kinematics.cpp)
ament_target_dependencies(
  turtlesim_kinematics
  rclcpp
  turtlesim
)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(
  simple_tf_kinematics
  rclcpp
  geometry_msgs
  tf2_ros
)

install(TARGETS
  turtlesim_kinematics
  simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
