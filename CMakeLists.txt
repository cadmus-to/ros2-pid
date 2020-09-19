cmake_minimum_required(VERSION 3.5)
project(pid)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

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

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces.

# messages
# set(msg_files
#   "msg/Triangle.msg"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   # ${msg_files}
#   DEPENDENCIES 
#     builtin_interfaces
#   ADD_LINTER_TESTS
# )

ament_export_dependencies(rosidl_default_runtime)

# Controller node.

add_executable(controller_node src/config.cpp src/pid.cpp src/controller.cpp src/controller_node.cpp)

ament_target_dependencies(controller_node 
  rclcpp)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

# rosidl_target_interfaces(controller_node
#   ${PROJECT_NAME} rosidl_typesupport_cpp)



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

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
