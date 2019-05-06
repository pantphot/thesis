/*
*   position_estimation.cpp
*   Author: Pantelis Photiou
*   Created: Mar 2019
*   Initializes a ROS 2 node which subscribes to a region of interest topic
*   and performs position estimation of detected face using the position of the
*   camera as the reference point. Publishes on "detection_pose" topic.
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "target_publisher.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include "rclcpp/duration.hpp"

Target_Publisher::Target_Publisher ()
: Node ("target_publisher"),
tfBuffer(),
msg_out(),
tf2_listener(tfBuffer)
{
  //   // Initialize publisher that publishes the pose
  pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "move_base_simple/goal", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the detected_point.
  std::cerr << "Subscribing to topic '/detected_pose'" << std::endl;
  sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "detected_pose", std::bind(&Target_Publisher::callback, this,  std::placeholders::_1),rmw_qos_profile_default);

  // Initialize clock to timestamp the output messages
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  }
  //
  // Target_Publisher::~Target_Publisher(){}
  //
  // Callback receiving the point_stamped message
  void Target_Publisher::callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    translate(msg, this->get_logger());
  }

  // Receive region_of_interest and estimate person position
  void Target_Publisher::translate(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg, rclcpp::Logger logger)
  {
    geometry_msgs::msg::PoseStamped temp;
    temp = *msg;
    // rclcpp::duration dur;
    try{

        tfBuffer.transform(*msg,msg_out,target_fr);
        if (msg_out.pose.position.y<100000.0){
        msg_out.pose.position.z=0;
        msg_out.header.stamp = clock -> now();
        pub -> publish(msg_out);
      }
    }
    catch (tf2::LookupException e)
    {std::cout << e.what() << '\n';}

  }

  #include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(Target_Publisher, rclcpp::Node)