cmake_minimum_required(VERSION 3.5)

project(monitorcpp)

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
#find_package(rmw_implementation REQUIRED)
##force opensplice implementation
#find_package(rmw_opensplice_cpp REQUIRED)
#find_package(rmw_fastrtps_cpp REQUIRED)
#find_package(ros2cli REQUIRED)
ament_package()

include_directories(include)

add_executable(monitorcpp
  src/monitor.cpp
  src/options.cpp
  src/throughput.cpp
)
ament_target_dependencies(monitorcpp
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rmw")

install(TARGETS
  monitorcpp
  DESTINATION lib/${PROJECT_NAME})
