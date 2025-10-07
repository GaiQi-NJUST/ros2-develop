cmake_minimum_required(VERSION 3.8)
project(msg_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(msg_publisher src/msg_publisher.cpp)
ament_target_dependencies(msg_publisher rclcpp std_msgs)

add_executable(msg_subscriber src/msg_subscriber.cpp)
ament_target_dependencies(msg_subscriber rclcpp std_msgs)

install(TARGETS
  msg_publisher
  msg_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()