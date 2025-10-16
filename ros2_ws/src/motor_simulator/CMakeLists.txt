cmake_minimum_required(VERSION 3.8)
project(motor_simulator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    include/
    ${EIGEN3_INCLUDE_DIRS}
)

# Define executables first
add_executable(motor_simulator_node 
  src/motor_simulator_node.cpp
)

add_executable(motor_controller_node 
  src/motor_controller.cpp
)

# Now set dependencies for each target
ament_target_dependencies(motor_simulator_node 
  rclcpp 
  std_msgs 
  geometry_msgs
)

ament_target_dependencies(motor_controller_node 
  rclcpp 
  std_msgs 
  geometry_msgs
)

# Add Eigen linking
if(TARGET Eigen3::Eigen)
  target_link_libraries(motor_simulator_node Eigen3::Eigen)
  target_link_libraries(motor_controller_node Eigen3::Eigen)
endif()

# Install targets
install(TARGETS
  motor_simulator_node
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install scripts if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY
    scripts/
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()