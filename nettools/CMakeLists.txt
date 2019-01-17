cmake_minimum_required(VERSION 3.5)

project(nettools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(nettools_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

ament_python_install_package(scripts/)

add_executable(latency
  src/latency.cpp
  src/options.cpp
)

ament_target_dependencies(latency
  "nettools_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rmw"
)

add_executable(throughput
  src/options.cpp
  src/throughput.cpp
  )

ament_target_dependencies(throughput
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rcl")

install(TARGETS
  latency
  throughput
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/nettools_plotter.py
  DESTINATION bin)


ament_package()
