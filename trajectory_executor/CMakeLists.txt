cmake_minimum_required(VERSION 3.8)
project(trajectory_executor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
include_directories(include)

add_executable(trajectory_executor 
  src/trajectory_executor.cpp
)

ament_target_dependencies(trajectory_executor
  rclcpp
  geometry_msgs
  std_msgs
)

# Install executable
install(TARGETS
  trajectory_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# data files
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
