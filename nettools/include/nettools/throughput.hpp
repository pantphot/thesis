#ifndef __NETTOOLS_THROUGHPUT_HPP
#define __NETTOOLS_THROUGHPUT_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/float64.hpp"

    class Throughput : public rclcpp::Node {
  public:
    Throughput(const std::string msg_type, const std::string topic,rmw_qos_profile_t custom_qos_profile);
    void callback(const std::shared_ptr<rmw_serialized_message_t> msg);
    void receive_msg(const std::shared_ptr<rmw_serialized_message_t>msg,rclcpp::Logger logger);
    ~Throughput();

    size_t count;
    std::string msg_type;
    std::string topic;
    rmw_qos_profile_t custom_qos_profile;
    rclcpp::TimerBase::SharedPtr periodic_timer;
    long int buffer;
    rclcpp::Subscription<rmw_serialized_message_t>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    double throughput;
    std_msgs::msg::Float64 throughput_avg;
  };


#endif
