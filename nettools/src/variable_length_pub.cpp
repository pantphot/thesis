/*
*   variable_length_pub.cpp
*   Author: Pantelis Photiou
*   Created: Jan 2019
*   Implements a ROS 2 node which publishes variable length char arrays,
*   on topic variable_length, used for tests.
*/
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "rcl/time.h"
#include "nettools/var_len_options.hpp"
#include "nettools_msgs/msg/byte_array.hpp"

//Fill message header and data except timestamp
void fill_message(
  int frame_id, int step, nettools_msgs::msg::ByteArray::SharedPtr msg)
{
  size_t size = step * 1024 *frame_id -24;
  size_t previous_size = step * 1024 * (frame_id-1) -24;
  msg->data.resize(size);
  for (uint x = previous_size ; x < size - 1 ; x++){
    msg->data[x] = 0;
  }
  msg->header.frame_id = std::to_string(frame_id);
}
int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  int freq = 30;
  int step = 1024;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);


  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure parameters with command line options.
  if (!parse_command_options(
      argc, argv, &depth, &reliability_policy, &history_policy, &freq, &step))
  {
    return 0;
  }

  // Initialize a ROS 2 node to publish variable length byte arrays.
  auto node = rclcpp::Node::make_shared("variable_length_pub");
  rclcpp::Logger node_logger = node->get_logger();

  // Set quality of service profile based on command line options.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = depth;
  custom_qos_profile.reliability = reliability_policy;
  custom_qos_profile.history = history_policy;

  RCLCPP_INFO(node_logger, "Publishing data on topic variable_length");

  //Create publisher with custom QoS profile
  auto pub = node->create_publisher<nettools_msgs::msg::ByteArray>("variable_length",custom_qos_profile);

  rclcpp::WallRate loop_rate(freq);

  //Initialize a shared pointer to a ByteArray message.
  auto msg = std::make_shared<nettools_msgs::msg::ByteArray>();

  int i = 1;

  // Main event loop will spin until the ucser presses CTRL -C to exit.
  while (rclcpp::ok()) {
    fill_message(i,step,msg);
    RCLCPP_INFO(node_logger, "Publishing message #%zd", i);
    msg->header.stamp = clock->now();
    std::cout << msg->data.size() << '\n';
    pub->publish(msg);
    ++i;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  // rclcpp::spin(std::make_shared<CalculateStatistics<MsgType>>(topic, custom_qos_profile));
  // }
  // else{
  //   std::cout << "/* Message type not supported */" << std::endl;
  //   exit(1);
  // }
  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();
  return 0;
}
