cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(direction_service_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# robot_patrol
add_executable(robot_patrol_node src/patrol_node.cpp)

ament_target_dependencies(robot_patrol_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros
)

add_library(robot_patrol_library
  src/patrol.cpp
)

ament_target_dependencies(robot_patrol_library
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros
)

target_link_libraries(robot_patrol_node
  robot_patrol_library
)

# direction_service
add_executable(direction_service_node src/direction_service_node.cpp)

ament_target_dependencies(direction_service_node
  direction_service_msg
  rclcpp
  sensor_msgs
  std_msgs
)

add_library(direction_service_lib
  src/direction_service.cpp
)

ament_target_dependencies(direction_service_lib
  direction_service_msg
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(direction_service_node
  direction_service_lib
)

# test_service
add_executable(test_service_executable src/test_service.cpp)

ament_target_dependencies(test_service_executable
  direction_service_msg
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  direction_service_node
  robot_patrol_node
  test_service_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()