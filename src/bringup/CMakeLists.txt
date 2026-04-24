cmake_minimum_required(VERSION 3.8)
project(bringup)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY config launch
DESTINATION share/${PROJECT_NAME})

ament_package()
