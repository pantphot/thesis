cmake_minimum_required(VERSION 3.5)

project(face_detection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include)

#ament_python_install_package(scripts/)

add_executable(detection_publisher
  src/detection_publisher.cpp
  src/options.cpp
  )

ament_target_dependencies(detection_publisher
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

add_executable(detection_subscriber
  src/detection_subscriber.cpp
  src/options.cpp
  )

ament_target_dependencies(detection_subscriber
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

install(TARGETS
  detection_publisher
  detection_subscriber
  DESTINATION lib/${PROJECT_NAME})


ament_package()
