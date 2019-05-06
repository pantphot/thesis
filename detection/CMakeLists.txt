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
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)
#
find_package(composition REQUIRED)
find_package(class_loader REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

#find_package(ament_cmake_python REQUIRED)
#find_package(python_cmake_module REQUIRED)

include_directories(include)

ament_python_install_package(scripts/)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

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

#add_executable(position_estimation
#  src/position_estimation.cpp
#  )

#ament_target_dependencies(position_estimation
#  "rclcpp"
#  "nettools_msgs"
#  "geometry_msgs"
#)

add_library(position_estimation_component SHARED
  src/position_estimation.cpp)
target_compile_definitions(position_estimation_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(position_estimation_component
  "class_loader"
  "rclcpp"
  "nettools_msgs"
  "geometry_msgs")
rclcpp_register_node_plugins(position_estimation_component "Position_Estimator")
set(node_plugins "${node_plugins}Position_Estimator;$<TARGET_FILE:position_estimation_component>\n")

#add_executable(target_publisher
#  src/target_publisher.cpp
#  )


#ament_target_dependencies(target_publisher
#  "rclpy"
#  "tf2_ros"
#  "geometry_msgs"
#  "tf2"
#  "tf2_geometry_msgs"
#  "kdl_parser"
#  "orocos_kdl"

#  )

add_library(target_publisher_component SHARED
  src/target_publisher.cpp)
target_compile_definitions(target_publisher_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(target_publisher_component
  "class_loader"
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "kdl_parser"
  "orocos_kdl")
rclcpp_register_node_plugins(target_publisher_component "Target_Publisher")
set(node_plugins "${node_plugins}Target_Publisher;$<TARGET_FILE:target_publisher_component>\n")

install(TARGETS
  detection
  #position_estimation
  #target_publisher
  DESTINATION lib/${PROJECT_NAME})


#launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

  # since the package installs libraries without exporting them
  # it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

install(TARGETS
  target_publisher_component
  position_estimation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
