cmake_minimum_required(VERSION 3.8)
project(hardware_system_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# Nodos
add_executable(hardware_emulator_node src/hardware_emulator_node.cpp)
ament_target_dependencies(hardware_emulator_node rclcpp custom_interfaces)

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp custom_interfaces)

add_executable(operator_node src/operator_node.cpp)
ament_target_dependencies(operator_node rclcpp custom_interfaces)

install(TARGETS
  hardware_emulator_node
  monitor_node
  operator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()