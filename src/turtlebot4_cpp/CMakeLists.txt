cmake_minimum_required(VERSION 3.8)
project(turtlebot4_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlebot4_node_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(turtlebot4_cpp_node src/turtlebot4_cpp_node.cpp)
ament_target_dependencies(turtlebot4_cpp_node rclcpp irobot_create_msgs rclcpp_action sensor_msgs geometry_msgs)
target_include_directories(turtlebot4_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS turtlebot4_cpp_node
#   DESTINATION lib/${PROJECT_NAME})

add_executable(turtlebot4_main_node src/turtlebot4_main_node.cpp)
ament_target_dependencies(turtlebot4_main_node rclcpp irobot_create_msgs rclcpp_action sensor_msgs geometry_msgs)
target_include_directories(turtlebot4_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_main_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(turtlebot4_goal_node src/turtlebot4_goal_node.cpp)
ament_target_dependencies(turtlebot4_goal_node rclcpp irobot_create_msgs rclcpp_action sensor_msgs geometry_msgs turtlebot4_node_interfaces)
target_include_directories(turtlebot4_goal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_goal_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(turtlebot4_drive_node src/turtlebot4_drive_node.cpp)
ament_target_dependencies(turtlebot4_drive_node rclcpp irobot_create_msgs rclcpp_action sensor_msgs geometry_msgs turtlebot4_node_interfaces)
target_include_directories(turtlebot4_drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_drive_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(turtlebot4_rotate_node src/turtlebot4_rotate_node.cpp)
ament_target_dependencies(turtlebot4_rotate_node rclcpp irobot_create_msgs rclcpp_action sensor_msgs geometry_msgs turtlebot4_node_interfaces)
target_include_directories(turtlebot4_rotate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_rotate_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  turtlebot4_drive_node
  turtlebot4_rotate_node
  turtlebot4_goal_node
  turtlebot4_main_node
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
