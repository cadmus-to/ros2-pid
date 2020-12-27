cmake_minimum_required(VERSION 3.5)
project(jlb_pid)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)
endif()

# Find dependencies

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install library 

install(
  DIRECTORY include/
  DESTINATION include
)

# LIBRARY.
## Create library for source code
add_library(${PROJECT_NAME}_core src/config.cpp src/pid.cpp src/controller.cpp)

## Link deps
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp)

  ## Link inclusions  
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_core
  DESTINATION lib)

# NODES.
# Use the library as input for the node
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${PROJECT_NAME}_core)

ament_target_dependencies(controller_node 
  rclcpp)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS controller_node
  EXPORT controller_node
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies

ament_export_dependencies(
  rclcpp
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME}_core)

# NOTE: Export not needed, executable should be found automatically
# ament_export_targets(controller_node)


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
