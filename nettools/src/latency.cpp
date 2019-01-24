#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "rcl/time.h"
#include <cmath>
#include "sensor_msgs/msg/image.hpp"
#include "nettools/options.hpp"
#include "nettools/latency.hpp"
#include "nettools_msgs/msg/byte_array.hpp"

// #include "nettools_msgs/msg/statistics_measurements.hpp"
// #include "nettools_msgs/msg/topic_statistics.hpp"
// enum MsgType {
//   Image, ByteArray
// };
//
// class Msg {
// public:
//   static Msg* Create(MsgType type);
// };
// class Image : public Msg{
//   return sensor_msgs::msg::Image;
// }
// class ByteArray : public Msg{
//   return nettools_msgs::msg::ByteArray;
// }
//
// Msg* Msg::Create(MsgType,type){
//   if (type == 'image')
//     return new Image();
//   else if (type == 'bytearray')
//     return new ByteArray();
//   else return NULL;
// }

template<typename T>
CalculateStatistics<T>::CalculateStatistics(const std::string topic,rmw_qos_profile_t custom_qos_profile)
: Node("latency"),
topic(topic),
custom_qos_profile(custom_qos_profile),
n_msgs_received(0),
latency_avg_last(0),
latency_last(0),
received_msg_id(0),
current_msg(0),
frequency_avg_last(0),
latest_sample()
{
  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  pub = this->create_publisher<nettools_msgs::msg::TopicStatistics>("topic_statistics",rmw_qos_profile_default);
  //******************************************************************************************************************
  sub = this->create_subscription<T>(
      topic.c_str(), std::bind(&CalculateStatistics::callback, this,  std::placeholders::_1),custom_qos_profile);
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

}
template<typename T>
CalculateStatistics<T>::~CalculateStatistics(){}
// *******************************************************************************************************************
template<typename T>
void CalculateStatistics<T>::callback(const std::shared_ptr<T> msg)
{
    receive_msg(msg, this->get_logger());
}
// ********************************************************************************************************************
template<typename T>
void CalculateStatistics<T>::receive_msg(const std::shared_ptr<T> msg,rclcpp::Logger logger)
{
  auto time_received = clock->now();
  received_msg_id = std::stoi (msg->header.frame_id,nullptr,10);
  RCLCPP_INFO(logger, "Received message #%d", received_msg_id);
  auto time_sent = rclcpp::Time( msg->header.stamp.sec, msg->header.stamp.nanosec,RCL_SYSTEM_TIME);
  sample(time_received,time_sent,received_msg_id,logger);
}
template<typename T>
void CalculateStatistics<T>::sample(const rclcpp::Time time_received, const rclcpp::Time time_sent, int received_msg_id, rclcpp::Logger logger){
  n_msgs_received+=1;
  //Compute the latency
  // auto begin_calc = clock->now();
  msg_out.latency.val = time_received.nanoseconds() - time_sent.nanoseconds();
  msg_out.latency.val =  RCL_NS_TO_MS(msg_out.latency.val);
  latency_avg_last = msg_out.latency.avg;
  msg_out.latency.avg += (msg_out.latency.val - msg_out.latency.avg )/double(n_msgs_received);
  msg_out.latency.std = std::sqrt(((n_msgs_received - 1) * std::pow(msg_out.latency.std,2) + (msg_out.latency.val - msg_out.latency.avg) * (msg_out.latency.val - latency_avg_last)) / n_msgs_received);
  if (n_msgs_received==1) msg_out.latency.min = msg_out.latency.val;
  msg_out.latency.min = (msg_out.latency.val < msg_out.latency.min)?(msg_out.latency.val):(msg_out.latency.min);
  msg_out.latency.max = (msg_out.latency.val > msg_out.latency.max)?(msg_out.latency.val):(msg_out.latency.max);

  // Interarrival jitter https://www.ietf.org/rfc/rfc3550.txt (mean deviation) packet length matters ,https://tools.ietf.org/html/rfc1889#appendix-A.8 example usage
  //          J(i) = J(i-1) + (|D(i-1,i)| - J(i-1))/16
  if (n_msgs_received > 1){
    auto d = msg_out.latency.val - latency_last;
    latency_last = msg_out.latency.val;
    if(d<0) d = -d;
    msg_out.jitter +=  (double(d) - msg_out.jitter)/16;// 16 noise reduction ratio
  }
  // If best effort calculate message loss
  if (custom_qos_profile.reliability == 2){
    if (n_msgs_received == 1){
      current_msg = received_msg_id;
    }
    else{
      msg_out.msg_loss += received_msg_id - current_msg - 1;
      current_msg = received_msg_id;
    }
  }

  // Compute receiving frequencyuency
  if (n_msgs_received < 2){
    msg_out.frequency.val = 0.0;
    latest_sample = time_received;
  }
  else if (n_msgs_received == 2){
    msg_out.frequency.val = (1.0/ RCL_NS_TO_S(double((time_received.nanoseconds() - latest_sample.nanoseconds()))));
    latest_sample = time_received;
    msg_out.frequency.avg = msg_out.frequency.val;
    msg_out.frequency.min = msg_out.frequency.val;
    msg_out.frequency.max = msg_out.frequency.val;
  }
  else{
    msg_out.frequency.val = (1.0/ RCL_NS_TO_S(double((time_received.nanoseconds() - latest_sample.nanoseconds()))));
    latest_sample = time_received;
    frequency_avg_last = msg_out.frequency.avg;
    msg_out.frequency.avg += (msg_out.frequency.val - msg_out.frequency.avg)/double(n_msgs_received - 1);
    msg_out.frequency.std =  std::sqrt(((n_msgs_received - 2) * std::pow(msg_out.frequency.std,2) + (msg_out.frequency.val - msg_out.frequency.avg) * (msg_out.frequency.val - frequency_avg_last)) / (n_msgs_received -1) );
    msg_out.frequency.min = (msg_out.frequency.val < msg_out.frequency.min) ? (msg_out.frequency.val) : (msg_out.frequency.min);
    msg_out.frequency.max = (msg_out.frequency.val > msg_out.frequency.max) ? (msg_out.frequency.val) : (msg_out.frequency.max);
  }


  // RCLCPP_INFO(logger, "Latency %lf ms", latency);
  // RCLCPP_INFO(logger, "Latency_avg %lf ms", msg_out.latency.avg);
  // // RCLCPP_INFO(logger, "Latency_avg %lf ms",latency_avg);
  // RCLCPP_INFO(logger, "Latency_std %lf ", msg_out.latency.std);
  // RCLCPP_INFO(logger, "Latency_min %lf ms", msg_out.latency.min);
  // RCLCPP_INFO(logger, "Latency_max %lf ms", msg_out.latency.max);
  // RCLCPP_INFO(logger, "Jitter %lf ", msg_out.jitter);
  // // RCLCPP_INFO(logger, "Message Loss %d ",msg_out.msg_loss);
  // RCLCPP_INFO(logger, "frequencyuency %lf Hz", frequency);
  // RCLCPP_INFO(logger, "Average frequencyuency %lf Hz", msg_out.frequency.avg);
  // RCLCPP_INFO(logger, "Std frequencyuency %lf Hz", msg_out.frequency.std);
  // RCLCPP_INFO(logger, "Min frequencyuency %lf Hz ", msg_out.frequency.min);
  // RCLCPP_INFO(logger, "Max frequencyuency %lf Hz", msg_out.frequency.max);
  pub->publish(msg_out);
  // auto done_calc = clock->now();
  // auto calc_time = done_calc.nanoseconds() - begin_calc.nanoseconds();
  // RCLCPP_INFO(logger, "Time for calculations %ld ns", calc_time);

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

  // Create Node based on topic type
  if (msg_type == "image"){
    typedef sensor_msgs::msg::Image MsgType;
    rclcpp::spin(std::make_shared<CalculateStatistics<MsgType>>(topic, custom_qos_profile));
  }
  else if (msg_type == "bytearray")
  {
    typedef nettools_msgs::msg::ByteArray MsgType;
    rclcpp::spin(std::make_shared<CalculateStatistics<MsgType>>(topic, custom_qos_profile));
  }
  else{
    std::cout << "/* Message type not supported */" << std::endl;
    exit(1);
  }
  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();
  return 0;
}
