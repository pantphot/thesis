/*
*   roi_compactor.cpp
*   Author: Pantelis Photiou
*   Created: May 2019
*   Initializes a ROS 2 node which subscribes to a region of interest topic
*   tranforms the RoiWithHeader.msg to RoiWithHeaderCompact.msg and publishes it on
*   "/compact_roi" topic.
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "roi_compactor.hpp"
#include "options.hpp"
#include <string>


Roi_Compactor::Roi_Compactor (rmw_qos_profile_t custom_qos_profile)
: Node ("roi_compactor"),
custom_qos_profile(custom_qos_profile)

{
  //   // Initialize publisher that publishes the compact roi
  pub = this->create_publisher<nettools_msgs::msg::RoiWithHeaderCompact>(
    "compact_roi", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the RoiWithHeader message.
  std::cerr << "Subscribing to topic '/region_of_interest'" << std::endl;
  sub = this->create_subscription<nettools_msgs::msg::RoiWithHeader>(
    "region_of_interest", std::bind(&Roi_Compactor::callback, this,  std::placeholders::_1),custom_qos_profile);

  }

  Roi_Compactor::~Roi_Compactor(){}

  // Callback receiving the region of interest
  void Roi_Compactor::callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg)
  {
    transform_msg(msg, this->get_logger());
  }

  // Receive region_of_interest and transform to compact message
  void Roi_Compactor::transform_msg(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger)
  {
    RCLCPP_INFO(logger, "Received msg # %s ",msg->header.frame_id.c_str());
    msg_out.sec = msg->header.stamp.sec;
    msg_out.nanosec = msg->header.stamp.nanosec;
    msg_out.frame_id = msg->header.frame_id;
    msg_out.x_offset = msg->roi.x_offset;
    msg_out.y_offset = msg->roi.y_offset;
    msg_out.height = msg->roi.height;
    msg_out.width = msg->roi.width;
    msg_out.image_width = msg->image_width;
    msg_out.image_height = msg->image_height;
    msg_out.do_rectify = msg->roi.do_rectify;

    pub -> publish(msg_out);

  }

  int main(int argc, char * argv[])
  {
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    bool show_camera = false;
    bool body = false;
    std::string topic("region_of_interest");
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure parameters with command line options.
    if (!parse_command_options(
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &body, &topic))
    {
      return 0;
    }

    // Set the parameters of the quality of service profile. Initialize as the default profile.

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = depth;
    custom_qos_profile.reliability = reliability_policy;
    custom_qos_profile.history = history_policy;
    // Create node and spin
    rclcpp::spin(std::make_shared<Roi_Compactor>(custom_qos_profile));

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
  }
