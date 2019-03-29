#ifndef __POSITION_ESTIMATION_POSITION_ESTIMATION_HPP
#define __POSITION_ESTIMATION_POSITION_ESTIMATION_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "nettools_msgs/msg/roi_with_header.hpp"
  // #include "rclcpp/clock.hpp"
  // #include "rclcpp/time.hpp"
  // #include "rclcpp/time_source.hpp"

  class Position_Estimator : public rclcpp::Node {
    public:
      Position_Estimator(rmw_qos_profile_t custom_qos_profile);
      void callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg);
      void estimate(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger);
      ~Position_Estimator();

      rclcpp::Subscription<nettools_msgs::msg::RoiWithHeader>::SharedPtr sub;
      rmw_qos_profile_t custom_qos_profile;
      uint image_width;
      double face_height;
      double focal_length;
      double hfov ;
      double thita;
      double distance;
      double displacement;
      double phi;
      double x;
      double z;
  };

#endif
