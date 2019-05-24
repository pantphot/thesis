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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(nettools_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)

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
  "nettools_msgs"
  "rmw")

add_executable(variable_length_pub
  src/var_len_options.cpp
  src/variable_length_pub.cpp
  )

ament_target_dependencies(variable_length_pub
  "rclcpp"
  "nettools_msgs"
  "rmw")

install(TARGETS
  latency
  throughput
  variable_length_pub
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/nettools_plotter.py
  scripts/variable_length_pub.py
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
