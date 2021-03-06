#ifndef __NETTOOLS_LATENCY_HPP
#define __NETTOOLS_LATENCY_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "nettools_msgs/msg/statistics_measurements.hpp"
  #include "nettools_msgs/msg/topic_statistics.hpp"

  template<class T>
  class CalculateStatistics : public rclcpp::Node {
    public:
      CalculateStatistics(const std::string topic,rmw_qos_profile_t custom_qos_profile);
      void callback(const std::shared_ptr<T> msg);
      void sample(const rclcpp::Time time_received, const rclcpp::Time time_sent,int received_msg_id, rclcpp::Logger logger);
      void receive_msg(const std::shared_ptr<T> msg, rclcpp::Logger logger);
      typename rclcpp::Subscription<T>::SharedPtr sub;
      rclcpp::Publisher<nettools_msgs::msg::TopicStatistics>::SharedPtr pub;
      ~CalculateStatistics();

      std::string msg_type;
      std::string topic;
      rmw_qos_profile_t custom_qos_profile;
      size_t n_msgs_received;
      nettools_msgs::msg::TopicStatistics msg_out;
      double latency_avg_last;
      double latency_last;
      rclcpp::Clock::SharedPtr clock;
      int received_msg_id;
      int current_msg;
      double frequency_avg_last;
      rclcpp::Time latest_sample;
  };

#endif
