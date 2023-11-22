cmake_minimum_required(VERSION 3.5)
project(tutorial_rclcpp_pub_sub)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

# To include any dependent messages
find_package(std_msgs REQUIRED)

# To generate custom interface
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TutorialClient.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

#For each executable
add_executable(rclcpp_pub_sub src/rclcpp_pub_sub.cpp)
ament_target_dependencies(rclcpp_pub_sub rclcpp std_msgs)
rosidl_target_interfaces(rclcpp_pub_sub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(rclcpp_pub src/rclcpp_pub.cpp)
ament_target_dependencies(rclcpp_pub rclcpp std_msgs)
rosidl_target_interfaces(rclcpp_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_include_directories(rclcpp_pub_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  rclcpp_pub_sub
  rclcpp_pub
  DESTINATION lib/${PROJECT_NAME})

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
