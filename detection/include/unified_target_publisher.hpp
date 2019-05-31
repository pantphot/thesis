#ifndef __UNIFIED_TARGET_PUBLISHER_UNIFIED_TARGET_PUBLISHER_HPP
#define __UNIFIED_TARGET_PUBLISHER_UNIFIED_TARGET_PUBLISHER_HPP
#include "rclcpp/rclcpp.hpp"
#include "nettools_msgs/msg/roi_with_header.hpp"
#include "nettools_msgs/msg/point_header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/impl/utils.h>

  class Unified_Target_Publisher : public rclcpp::Node {
    public:
      Unified_Target_Publisher(rmw_qos_profile_t custom_qos_profile);
      void callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg);
      void estimate(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger);
      ~Unified_Target_Publisher();

      rclcpp::Subscription<nettools_msgs::msg::RoiWithHeader>::SharedPtr sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_real;
      rclcpp::Publisher<nettools_msgs::msg::PointHeader>::SharedPtr pub_point;

      geometry_msgs::msg::PoseStamped msg_out_actual;
      geometry_msgs::msg::PoseStamped msg_out;
      nettools_msgs::msg::PointHeader point_msg;

      rmw_qos_profile_t custom_qos_profile;
      // rclcpp::Clock::SharedPtr clock;
      rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

      std::string target_fr = "map";
      tf2_ros::Buffer tfBuffer;
      tf2::Duration dur;
      tf2_ros::TransformListener tf2_listener;

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
      double r;
      double yaw;
  };

#endif
