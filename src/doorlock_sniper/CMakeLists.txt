cmake_minimum_required(VERSION 3.8)
project(doorlock_sniper)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Protobuf
find_package(Protobuf REQUIRED)
# MQTT
find_library(PAHO_MQTTPP_LIB NAMES paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB NAMES paho-mqtt3a paho-mqtt3c)

set(PROTO_FILES
  proto/video_stream.proto
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 查找 GStreamer 1.0
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED 
  gstreamer-1.0>=1.4 
  gstreamer-app-1.0>=1.4
)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VideoPacket.msg
  DEPENDENCIES std_msgs
)
# 包含目录
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})

# 创建 Composable Node 共享库
add_library(video_encoder_node SHARED
  src/video_encoder_node.cpp
  ${PROTO_SRCS}
)

target_compile_features(video_encoder_node PUBLIC cxx_std_17)

ament_target_dependencies(video_encoder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  std_msgs
  OpenCV
)

# 链接 GStreamer 库 ， Protobuf 库 和 Paho MQTT 库
target_link_libraries(video_encoder_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${PAHO_MQTTPP_LIB}
  ${PAHO_MQTT_C_LIB}
)

# 关键：链接生成的消息类型支持（确保头文件生成顺序）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(video_encoder_node ${cpp_typesupport_target})

# 注册组件插件
rclcpp_components_register_nodes(video_encoder_node "doorlock_sniper::VideoEncoderNode")

# 安装
install(TARGETS video_encoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
