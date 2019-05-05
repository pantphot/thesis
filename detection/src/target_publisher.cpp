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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include "rclcpp/duration.hpp"

Target_Publisher::Target_Publisher (rmw_qos_profile_t custom_qos_profile)
: Node ("position_estimator"),
custom_qos_profile(custom_qos_profile),
tfBuffer()
// image_width(),
// image_height(),
// face_height(16.0),
// focal_length(),
// hfov(),
// vfov(),
// thita_x(),
// thita_y(),
// distance(),
// displacement_x(),
// displacement_y(),
// phi_x(),
// phi_y(),
// x(),
// z(),
// y()

{
  //   // Initialize publisher that publishes the pose
  pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "move_base_simple/goal", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the detected_point.
  std::cerr << "Subscribing to topic '/detected_point'" << std::endl;
  sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "detected_point", std::bind(&Target_Publisher::callback, this,  std::placeholders::_1),custom_qos_profile);


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
    geometry_msgs::msg::PoseStamped temp;
    temp = *msg;
    auto tf_listener = tf2_ros::TransformListener(tfBuffer);
    // rclcpp::duration dur;
    tf2_ros::BufferInterface::transform<geometry_msgs::msg::PoseStamped>(temp,"map",rclcpp::Duration(0.0) );

    //RCLCPP_INFO(logger, msg);
    // // RCLCPP_INFO(logger, "phi_x = %f deg", phi_x*57.3);
    // // RCLCPP_INFO(logger, "phi_y = %f deg", phi_y*57.3);
    // RCLCPP_INFO(logger, "x = %f cm" ,x);
    // RCLCPP_INFO(logger, "y = %f cm", y);
    // // RCLCPP_INFO(logger, "z_y = %f cm", z_y);
    // RCLCPP_INFO(logger, "z_x = %f cm", z);
    //



    msg_out.header.frame_id = "external_camera";
    msg_out.header.stamp = clock -> now();
    msg_out.point.y = -x/100;
    msg_out.point.z = y/100;
    msg_out.point.x = z/100;
    // msg_out.pose.position.y = -x/100;
    // msg_out.pose.position.z = y/100;
    // msg_out.pose.position.x = z/100;
    // msg_out.pose.orientation.x = 0.0;
    // msg_out.pose.orientation.y = 0.0;
    // msg_out.pose.orientation.z = 0.0;
    // msg_out.pose.orientation.w = 1.0;

    pub -> publish(msg_out);

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
