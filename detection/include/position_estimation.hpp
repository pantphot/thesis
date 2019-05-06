#ifndef __POSITION_ESTIMATION_POSITION_ESTIMATION_HPP
#define __POSITION_ESTIMATION_POSITION_ESTIMATION_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "nettools_msgs/msg/roi_with_header.hpp"
  #include "geometry_msgs/msg/pose_stamped.hpp"
  #include "rclcpp/clock.hpp"
  #include "rclcpp/time.hpp"
  #include "visibility_control.h"
  #include "rmw/qos_profiles.h"
  // #include "rclcpp/time_source.hpp"

  class Position_Estimator : public rclcpp::Node {
    public:
      COMPOSITION_PUBLIC
      Position_Estimator();
      void callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg);
      void estimate(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger);

      rclcpp::Subscription<nettools_msgs::msg::RoiWithHeader>::SharedPtr sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
      geometry_msgs::msg::PoseStamped msg_out;
      rclcpp::Clock::SharedPtr clock;

      uint image_width;
      uint image_height;
      double face_height;
      double focal_length;
      double hfov ;
      double vfov ;
      double thita_x;
      double thita_y;
      double distance;
      double displacement_x;
      double displacement_y;
      double phi_x;
      double phi_y;
      double x;
      double z;
      double y;
  };

#endif
