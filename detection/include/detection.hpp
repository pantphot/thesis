#ifndef __DETECTION_DETECTION_HPP
#define __DETECTION_DETECTION_HPP
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/image.hpp"
  #include "nettools_msgs/msg/roi_with_header.hpp"
  #include "opencv2/objdetect/objdetect.hpp"
  #include "opencv2/highgui/highgui.hpp"
  #include "opencv2/imgproc/imgproc.hpp"
  #include "rclcpp/clock.hpp"
  #include "rclcpp/time.hpp"
  #include "rclcpp/time_source.hpp"

  class Detector : public rclcpp::Node {
    public:
      Detector(const std::string topic,rmw_qos_profile_t custom_qos_profile, bool show_camera, bool body);
      int encoding2mat_type(const std::string & encoding);
      void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
      void detectAndDisplay(const std::shared_ptr<sensor_msgs::msg::Image> msg, rclcpp::Logger logger);
      ~Detector();

      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
      rclcpp::Publisher<nettools_msgs::msg::RoiWithHeader>::SharedPtr pub;
      rclcpp::Clock::SharedPtr clock ;
      std::string topic;
      rmw_qos_profile_t custom_qos_profile;
      nettools_msgs::msg::RoiWithHeader msg_out;
      bool show_camera;
      cv::CascadeClassifier cascade;
      bool body;
      size_t i;
<<<<<<< HEAD
      int current_msg;
      int msg_loss;
=======
      double min_face_size;
      double max_face_size;
>>>>>>> 7bc1a9a4a811e457d5ce1356fb3fe5995c8fc025
  };

#endif
