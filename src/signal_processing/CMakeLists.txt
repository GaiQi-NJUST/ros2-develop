cmake_minimum_required(VERSION 3.8)
project(signal_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# 1. 编译发布者节点
add_executable(signal_publisher src/signal_publisher.cpp)
ament_target_dependencies(signal_publisher rclcpp std_msgs geometry_msgs)

# 2. 编译订阅者节点
add_executable(signal_subscriber src/signal_subscriber.cpp)
ament_target_dependencies(signal_subscriber rclcpp std_msgs geometry_msgs)

# 3. 安装节点（让ROS2能找到可执行文件）
install(TARGETS
  signal_publisher
  signal_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
