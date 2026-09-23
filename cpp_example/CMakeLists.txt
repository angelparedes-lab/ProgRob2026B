cmake_minimum_required(VERSION 3.8)
project(cpp_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robx7_interfaces REQUIRED)


add_executable(test_node_cpp src/test_node.cpp)
ament_target_dependencies(test_node_cpp rclcpp)

add_executable(example_publisher_cpp src/example_publisher.cpp)
ament_target_dependencies(example_publisher_cpp rclcpp std_msgs)

add_executable(example_subscriber_cpp src/example_subscriber.cpp)
ament_target_dependencies(example_subscriber_cpp rclcpp std_msgs)

add_executable(custom_publisher_cpp src/custom_publisher.cpp)
ament_target_dependencies(custom_publisher_cpp rclcpp robx7_interfaces)

add_executable(custom_subscriber_cpp src/custom_subscriber.cpp)
ament_target_dependencies(custom_subscriber_cpp rclcpp robx7_interfaces)

add_executable(server_cpp src/example_service_server.cpp)
ament_target_dependencies(server_cpp rclcpp example_interfaces)

add_executable(client_cpp src/example_service_client.cpp)
ament_target_dependencies(client_cpp rclcpp example_interfaces)

add_executable(custom_server_cpp src/custom_service_server.cpp)
ament_target_dependencies(custom_server_cpp rclcpp robx7_interfaces)

add_executable(custom_client_cpp src/custom_service_client.cpp)
ament_target_dependencies(custom_client_cpp rclcpp robx7_interfaces)

add_executable(client2_cpp src/example_client.cpp)
ament_target_dependencies(client2_cpp rclcpp example_interfaces)

install(TARGETS
  test_node_cpp
  example_publisher_cpp
  example_subscriber_cpp
  custom_publisher_cpp
  custom_subscriber_cpp
  server_cpp
  client_cpp
  custom_client_cpp
  custom_server_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


