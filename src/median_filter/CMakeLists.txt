cmake_minimum_required(VERSION 3.8)
project(median_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable for the publisher node
add_executable(MF_publisher_node src/MF_publisher.cpp)
ament_target_dependencies(MF_publisher_node rclcpp std_msgs)

# Add executable for the subscriber node
add_executable(MF_subscriber_node src/MF_subscriber.cpp)
ament_target_dependencies(MF_subscriber_node rclcpp std_msgs)

# Install the executable nodes
install(TARGETS
  MF_publisher_node
  MF_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

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