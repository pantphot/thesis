/*
*   throughput.cpp
*   Author: Pantelis Photiou
*   Created: Jan 2019
*   Initializes a ROS 2 node which subcribes on certain topics and
*   calculates the throughput of given topic, which then publishes on topic
*   topic_statistics
*/
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "nettools/options.hpp"
#include <cstdio>
#include "nettools/throughput.hpp"
#include "nettools_msgs/msg/byte_array.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

Throughput::Throughput(const std::string msg_type, const std::string topic,rmw_qos_profile_t custom_qos_profile)
: Node("throughput"),
  // count(1),
  msg_type(msg_type),
  topic(topic),
  custom_qos_profile(custom_qos_profile),
  buffer(0),
  bufferMb(0)
  {

  //Create publisher
  pub = this->create_publisher<std_msgs::msg::Float64>("topic_statistics",rmw_qos_profile_default);

  // Create Subscription to topic
  if (msg_type == "image"){
  std::cout << "Creating image subscriber" << std::endl;
  sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic.c_str(), std::bind(&Throughput::callback, this,  std::placeholders::_1),custom_qos_profile);
  }
  else if (msg_type == "bytearray"){
  std::cout << "Creating bytearray subscriber" << std::endl;
  sub = this->create_subscription<nettools_msgs::msg::ByteArray>(
        topic.c_str(), std::bind(&Throughput::callback, this,  std::placeholders::_1),custom_qos_profile);
  }
  else
  {
    std::cout << "/* Message type not supported */" << std::endl;
    exit(1);
  }
  periodic_timer = this->create_wall_timer(
      1s,
      [this]()
      {
        throughput.data = double(buffer)/131072;
        //throughput.avg += (throughput.val - throughput.avg)/ this->count;
        pub->publish(throughput);
        // count +=1;
        buffer = 0;

      });
  }
Throughput::~Throughput(){}

// Callback Function Receiving data
void Throughput::callback(const std::shared_ptr<rmw_serialized_message_t> msg)
{
  buffer = buffer + msg->buffer_length;
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
  std::string msg_type;

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
  if (reliability_policy==1)
    std::cout << "/* Reliable */" << '\n';
  else
    std::cout << "/* Best effort */" << '\n';

  //Initialize a Throughput object (ROS node)
  rclcpp::spin(std::make_shared<Throughput>(msg_type, topic, custom_qos_profile));

  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();

  return 0;
}
