cmake_minimum_required(VERSION 3.8)
project(robx7_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_viewer src/camera_viewer.cpp)
ament_target_dependencies(camera_viewer rclcpp cv_bridge sensor_msgs)

target_include_directories(camera_viewer PRIVATE $
{OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_viewer ${OpenCV_LIBS})

install(TARGETS camera_viewer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
