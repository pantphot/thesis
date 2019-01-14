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
// #include "nettools_msgs/msg/statistics_measurements.hpp"
// #include "nettools_msgs/msg/topic_statistics.hpp"




CalculateStatistics::CalculateStatistics(const std::string msg_type, const std::string topic,rmw_qos_profile_t custom_qos_profile)
: Node("latency"),
msg_type(msg_type),
topic(topic),
custom_qos_profile(custom_qos_profile),
n_msgs_received(0),
latency(0),
latency_avg_last(0),
latency_last(0),
received_msg_id(0),
current_msg(0),
freq(0),
freq_avg_last(0),
latest_sample()
{
  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  std::cerr << "Subscribing to topic ' " << topic << "'" << std::endl;
  pub = this->create_publisher<nettools_msgs::msg::TopicStatistics>("topic_statistics",custom_qos_profile);
  sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic.c_str(), std::bind(&CalculateStatistics::callback, this,  std::placeholders::_1),custom_qos_profile);
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

}

CalculateStatistics::~CalculateStatistics(){}

void CalculateStatistics::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
    receive_msg(msg, this->get_logger());
}

void CalculateStatistics::receive_msg(const std::shared_ptr<sensor_msgs::msg::Image> msg,rclcpp::Logger logger)
{
  auto time_received = clock->now();
  received_msg_id = std::stoi (msg->header.frame_id,nullptr,10);
  RCLCPP_INFO(logger, "Received image #%d", received_msg_id);
  auto time_sent = rclcpp::Time( msg->header.stamp.sec, msg->header.stamp.nanosec,RCL_SYSTEM_TIME);
  sample(time_received,time_sent,received_msg_id,logger);
}

void CalculateStatistics::sample(const rclcpp::Time time_received, const rclcpp::Time time_sent, int received_msg_id, rclcpp::Logger logger){
  n_msgs_received+=1;
  //Compute the latency
  // auto begin_calc = clock->now();
  latency = time_received.nanoseconds() - time_sent.nanoseconds();
  latency =  RCL_NS_TO_MS(latency);
  latency_avg_last = msg_out.latency.avg;
  msg_out.latency.avg += (latency - msg_out.latency.avg )/double(n_msgs_received);
  msg_out.latency.std = std::sqrt(((n_msgs_received - 1) * std::pow(msg_out.latency.std,2) + (latency - msg_out.latency.avg) * (latency - latency_avg_last)) / n_msgs_received);
  if (n_msgs_received==1) msg_out.latency.min = latency;
  msg_out.latency.min = (latency < msg_out.latency.min)?(latency):(msg_out.latency.min);
  msg_out.latency.max = (latency > msg_out.latency.max)?(latency):(msg_out.latency.max);

  // Interarrival jitter https://www.ietf.org/rfc/rfc3550.txt (mean deviation) packet length matters ,https://tools.ietf.org/html/rfc1889#appendix-A.8 example usage
  //          J(i) = J(i-1) + (|D(i-1,i)| - J(i-1))/16
  if (n_msgs_received > 1){
    auto d = latency - latency_last;
    latency_last = latency;
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

  // Compute receiving frequency
  if (n_msgs_received < 2){
    freq = 0.0;
    latest_sample = time_received;
  }
  else if (n_msgs_received == 2){
    freq = (1.0/ RCL_NS_TO_S(double((time_received.nanoseconds() - latest_sample.nanoseconds()))));
    latest_sample = time_received;
    msg_out.freq.avg = freq;
    msg_out.freq.min = freq;
    msg_out.freq.max = freq;
  }
  else{
    freq = (1.0/ RCL_NS_TO_S(double((time_received.nanoseconds() - latest_sample.nanoseconds()))));
    latest_sample = time_received;
    freq_avg_last = msg_out.freq.avg;
    msg_out.freq.avg += (freq - msg_out.freq.avg)/double(n_msgs_received - 1);
    msg_out.freq.std =  std::sqrt(((n_msgs_received - 2) * std::pow(msg_out.freq.std,2) + (freq - msg_out.freq.avg) * (freq - freq_avg_last)) / (n_msgs_received -1) );
    msg_out.freq.min = (freq < msg_out.freq.min) ? (freq) : (msg_out.freq.min);
    msg_out.freq.max = (freq > msg_out.freq.max) ? (freq) : (msg_out.freq.max);
  }


  // RCLCPP_INFO(logger, "Latency %lf ms", latency);
  // RCLCPP_INFO(logger, "Latency_avg %lf ms", msg_out.latency.avg);
  // // RCLCPP_INFO(logger, "Latency_avg %lf ms",latency_avg);
  // RCLCPP_INFO(logger, "Latency_std %lf ", msg_out.latency.std);
  // RCLCPP_INFO(logger, "Latency_min %lf ms", msg_out.latency.min);
  // RCLCPP_INFO(logger, "Latency_max %lf ms", msg_out.latency.max);
  // RCLCPP_INFO(logger, "Jitter %lf ", msg_out.jitter);
  // // RCLCPP_INFO(logger, "Message Loss %d ",msg_out.msg_loss);
  // RCLCPP_INFO(logger, "frequency %lf Hz", freq);
  // RCLCPP_INFO(logger, "Average frequency %lf Hz", msg_out.freq.avg);
  // RCLCPP_INFO(logger, "Std frequency %lf Hz", msg_out.freq.std);
  // RCLCPP_INFO(logger, "Min frequency %lf Hz ", msg_out.freq.min);
  // RCLCPP_INFO(logger, "Max frequency %lf Hz", msg_out.freq.max);
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

  // Create Node, subsrciption...
  rclcpp::spin(std::make_shared<CalculateStatistics>(msg_type, topic, custom_qos_profile));
  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();
  return 0;
}
