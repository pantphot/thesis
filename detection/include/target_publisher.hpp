#ifndef __TARGET_PUBLISHER_TARGET_PUBLISHER_HPP
#define __TARGET_PUBLISHER_TARGET_PUBLISHER_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "geometry_msgs/msg/pose_stamped.hpp"
  #include "rclcpp/clock.hpp"
  #include "rclcpp/time.hpp"
  #include "tf2_ros/buffer.h"
  #include "tf2_ros/buffer_interface.h"
  #include <tf2_ros/transform_listener.h>

  class Target_Publisher : public rclcpp::Node {
    public:
      Target_Publisher(rmw_qos_profile_t custom_qos_profile);
      void callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
      void translate(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg, rclcpp::Logger logger);
      ~Target_Publisher();

      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
      geometry_msgs::msg::PoseStamped msg_out;
      rmw_qos_profile_t custom_qos_profile;
      // rclcpp::Clock::SharedPtr clock;
      rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

      tf2_ros::Buffer tfBuffer;
      std::string target_fr = "map";
      tf2_ros::TransformListener tf2_listener;

  };

#endif
