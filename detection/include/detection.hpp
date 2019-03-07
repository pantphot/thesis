#ifndef __DETECTION_DETECTION_HPP
#define __DETECTION_DETECTION_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/image.hpp"
  #include "sensor_msgs/msg/region_of_interest.hpp"
  #include "opencv2/objdetect/objdetect.hpp"
  #include "opencv2/highgui/highgui.hpp"
  #include "opencv2/imgproc/imgproc.hpp"

  class Detector : public rclcpp::Node {
    public:
      Detector(const std::string topic,rmw_qos_profile_t custom_qos_profile, bool show_camera, bool body);
      int encoding2mat_type(const std::string & encoding);
      void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
      void detectAndDisplay(const std::shared_ptr<sensor_msgs::msg::Image> msg, rclcpp::Logger logger);
      ~Detector();

      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
      rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr pub;
      std::string topic;
      rmw_qos_profile_t custom_qos_profile;
      sensor_msgs::msg::RegionOfInterest msg_out;
      bool show_camera;
      cv::CascadeClassifier cascade;
      bool body;
  };

#endif
