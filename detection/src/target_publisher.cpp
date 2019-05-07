/*
*   target_publisher.cpp
*   Author: Pantelis Photiou
*   Created: May 2019
*   Initializes a ROS 2 node which subscribes to a detected_pose topic
*   and performs frame transformation from external camera to map frame  Publishes on "move_base_simple/goal" topic a PoseStamped message.
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

Target_Publisher::Target_Publisher (rmw_qos_profile_t custom_qos_profile)
: Node ("target_publisher"),
custom_qos_profile(custom_qos_profile),
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
    "detected_pose", std::bind(&Target_Publisher::callback, this,  std::placeholders::_1),custom_qos_profile);

  // Initialize clock to timestamp the output messages
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  }
  //
  Target_Publisher::~Target_Publisher(){}
  //
  // Callback receiving the point_stamped message
  void Target_Publisher::callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    translate(msg, this->get_logger());
  }

  // Receive region_of_interest and estimate person position
  void Target_Publisher::translate(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg, rclcpp::Logger logger)
  {
    
    // rclcpp::duration dur;
    try{
    	tfBuffer.transform(*msg,msg_out,target_fr);
      	if (msg_out.pose.position.x < 10000.0){
      		msg_out.pose.position.z=0;
    		RCLCPP_INFO(logger,"Target Coordinates = (%lf , %lf)",msg_out.pose.position.x,msg_out.pose.position.y);
		    msg_out.header.stamp = clock -> now();
    		pub -> publish(msg_out);
   	 }
    }
    catch (tf2::LookupException e)
    {std::cout << e.what() << '\n';}

  }

  int main(int argc, char * argv[])
  {
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure parameters with command line options.

    // Set the parameters of the quality of service profile. Initialize as the default profile.

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // Create node and spin
    rclcpp::spin(std::make_shared<Target_Publisher>(custom_qos_profile));

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
  }
