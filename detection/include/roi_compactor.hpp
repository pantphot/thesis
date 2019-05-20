#ifndef __ROI_COMPACTOR_ROI_COMPACTOR_HPP
#define __ROI_COMPACTOR_ROI_COMPACTOR_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "nettools_msgs/msg/roi_with_header.hpp"
  #include "nettools_msgs/msg/roi_with_header_compact.hpp"

  class Roi_Compactor : public rclcpp::Node {
    public:
      Roi_Compactor(rmw_qos_profile_t custom_qos_profile);
      void callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg);
      void transform_msg(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger);
      ~Roi_Compactor();

      rclcpp::Subscription<nettools_msgs::msg::RoiWithHeader>::SharedPtr sub;
      rclcpp::Publisher<nettools_msgs::msg::RoiWithHeaderCompact>::SharedPtr pub;

      nettools_msgs::msg::RoiWithHeaderCompact msg_out;

      rmw_qos_profile_t custom_qos_profile;

  };

#endif
