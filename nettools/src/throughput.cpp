#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nettools/options.hpp"
#include <cstdio>
#include "nettools/throughput.hpp"
#include "nettools_msgs/msg/byte_array.hpp"

using namespace std::chrono_literals;

Throughput::Throughput(const std::string msg_type, const std::string topic,rmw_qos_profile_t custom_qos_profile)
: Node("throughput"),
  count(0),
  msg_type(msg_type),
  topic(topic),
  custom_qos_profile(custom_qos_profile),
  buffer(0),
  throughput(0)
  {

  //Create publisher
  pub = this->create_publisher<std_msgs::msg::Float64>("topic_statistics",rmw_qos_profile_default);

  // Create Subscription to topic
  // sub = this->create_subscription<sensor_msgs::msg::Image>(
  //       topic.c_str(), std::bind(&Throughput::callback, this,  std::placeholders::_1),custom_qos_profile);
  sub = this->create_subscription<nettools_msgs::msg::ByteArray>(
        topic.c_str(), std::bind(&Throughput::callback, this,  std::placeholders::_1),custom_qos_profile);

  periodic_timer = this->create_wall_timer(
      1s,
      [this]()
      {
        // RCLCPP_INFO(this->get_logger(), "in periodic_timer callback");

        throughput = double(buffer)/131072;
        throughput_avg.data = (count * throughput_avg.data + throughput) / (count + 1);
        RCLCPP_INFO(this->get_logger(), "Throughput = %lf Mbps",throughput_avg);

        pub->publish(throughput_avg);
        count +=1;
        buffer = 0;

      });
  }
Throughput::~Throughput(){}

// Callback Function Receiving data
void Throughput::callback(const std::shared_ptr<rmw_serialized_message_t> msg)
{
    receive_msg(msg, this->get_logger());
}

void Throughput::receive_msg(const std::shared_ptr<rmw_serialized_message_t>msg,rclcpp::Logger logger)
{
  buffer = buffer + msg->buffer_length;
  // if (count==1){
  RCLCPP_INFO(logger, "Received data of length %ld ",  msg->buffer_length);
  // };
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
  if (reliability_policy==1)
    std::cout << "/* Reliable */" << '\n';
  else
    std::cout << "/* Best effort */" << '\n';
  //Initialize a ROS node
  // auto node = rclcpp::Node::make_shared("throughput");
  // auto node = std::make_shared<Throughput>();
  //
  // //Declaration of callback function for receiving messages
  // auto callback = [&node](const std::shared_ptr<rmw_serialized_message_t>msg)
  // {
  //   receive_msg(msg, node->get_logger());
  // };

  // std::cerr << "Subscribing to topic ' " << topic << "'" << std::endl;
  // RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());

  // // Initialize a subscriber that will receive the ROS Image message to be displayed.
  // rclcpp::Subscription<rmw_serialized_message_t>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::Image>(
  //   topic.c_str(), callback, custom_qos_profile);
  //
  // std::cerr << "Spinning" << std::endl;

  //Initialize a Throughput object (ROS node)
  rclcpp::spin(std::make_shared<Throughput>(msg_type, topic, custom_qos_profile));

  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();

  return 0;
}
