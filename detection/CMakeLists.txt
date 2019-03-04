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
find_package(rmw REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include)

#ament_python_install_package(scripts/)

add_executable(detection
  src/detection.cpp
  src/options.cpp
  )

ament_target_dependencies(detection
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
#"rmw"
)
install(TARGETS
  detection
  DESTINATION lib/${PROJECT_NAME})

#install(PROGRAMS
#  scripts/nettools_plotter.py
#  scripts/variable_length_pub.py
#  DESTINATION lib/${PROJECT_NAME})


ament_package()
