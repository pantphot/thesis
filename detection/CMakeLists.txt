cmake_minimum_required(VERSION 3.5)

project(detection)

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
find_package(OpenCV REQUIRED)
find_package(nettools_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)

#find_package(ament_cmake_python REQUIRED)
#find_package(python_cmake_module REQUIRED)

include_directories(include)

ament_python_install_package(scripts/)

add_executable(detection
  src/detection.cpp
  src/options.cpp
  )

ament_target_dependencies(detection
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "nettools_msgs"
)

add_executable(position_estimation
  src/position_estimation.cpp
  )

ament_target_dependencies(position_estimation
  "rclcpp"
  "nettools_msgs"
  "geometry_msgs"
)

#add_executable(target_publisher
#  src/target_publisher.cpp
#  )


#ament_target_dependencies(target_publisher
#  "rclpy"
#  "tf2_ros"
#  "geometry_msgs"
#  "tf2_geometry_msgs"
#  )

install(TARGETS
  detection
  position_estimation
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/target_publisher.py
  DESTINATION lib/${PROJECT_NAME})

#launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
