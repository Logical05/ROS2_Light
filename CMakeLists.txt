cmake_minimum_required(VERSION 3.8)
project(ros2_light)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY description launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# # Include Cpp "include" directory
# include_directories(include)

# # Create Cpp executable
# add_executable(cpp_node src/cpp/cpp_node.cpp)
# ament_target_dependencies(cpp_node rclcpp)

# # Install Cpp executables
# install(
#   TARGETS cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_py)

# Install Python executables
install(
  PROGRAMS src/python/monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS src/python/teleop_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()