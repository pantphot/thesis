#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "monitorcpp/optionsmonitor.hpp"


// Message Callback
void receive_msg(const std::shared_ptr<rmw_serialized_message_t>msg,rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received data of length %d ", msg->buffer_length);
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  std::string topic;
  std::string msg_type ("Image");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure parameters with command line options.
  if (!parse_command_options(
      argc, argv, &depth, &reliability_policy, &history_policy, &msg_type, &topic))
  {
    return 0;
  }

  // Set quality of service profile based on command line options.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = depth;
  custom_qos_profile.reliability = reliability_policy;
  custom_qos_profile.history = history_policy;

  //Initialize a ROS node
  auto node = rclcpp::Node::make_shared("throughput");

  //Declaration of callback function for receiving messages
  auto callback = [&node](const std::shared_ptr<rmw_serialized_message_t>msg)
  {
    receive_msg(msg, node->get_logger());
  };

  std::cerr << "Subscribing to topic ' " << topic << "'" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());

  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  rclcpp::Subscription<rmw_serialized_message_t>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic.c_str(), callback, custom_qos_profile);

  std::cerr << "Spinning" << std::endl;
  rclcpp::spin(node);

  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();

  return 0;
}
