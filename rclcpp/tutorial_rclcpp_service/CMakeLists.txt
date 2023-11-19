cmake_minimum_required(VERSION 3.5)
project(tutorial_rclcpp_service)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_rclcpp_pub_sub REQUIRED)

# To generate custom interface
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/TutorialServiceAdd.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
 )

add_executable(rclcpp_service_server src/rclcpp_service_server.cpp)

# To inlcude tutorial_rclcpp_pub_sub message
ament_target_dependencies(
  rclcpp_service_server
  rclcpp
  std_msgs
  tutorial_rclcpp_pub_sub
)

target_include_directories(rclcpp_service_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  rclcpp_service_server
  DESTINATION lib/${PROJECT_NAME})

# To include custom message to current project
rosidl_target_interfaces(
  rclcpp_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
