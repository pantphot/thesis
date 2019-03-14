/*
*   detection.cpp
*   Author: Pantelis Photiou
*   Created: Mar 2019
*   Initializes a ROS 2 node which subscribes to an image topic
*   and performs face or body detection on received image. The node then publishes
*   the region of interest on topic region_of_interest.
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/region_of_interest.hpp"
#include "options.hpp"
#include "detection.hpp"

using namespace std;
using namespace cv;

// CascadeClassifier cascade;

Detector::Detector (const std::string topic,rmw_qos_profile_t custom_qos_profile, bool show_camera, bool body)
: Node ("detector"),
topic(topic),
custom_qos_profile(custom_qos_profile),
show_camera(show_camera),
body(body),
i(0),
current_msg(),
msg_loss(0)
{

  // Load the cascades
  auto cascade_name = "./src/detection/data/haarcascade_fullbody.xml";
  if (body != 1){
    cascade_name = "./src/detection/data/haarcascade_frontalface_alt.xml";
  }
  cascade.load( cascade_name );
  if( !cascade.load( cascade_name ) )
  {
    cout << "Error loading cascade!\n";
  }

  // Initialize an OpenCV named window.
  if (show_camera) {
    std::cerr << "Creating window" << std::endl;
    cvNamedWindow("detection", CV_WINDOW_AUTOSIZE);
    waitKey(1);
  }

  // Initialize publisher that publishes the ROI
  pub = this->create_publisher<nettools_msgs::msg::RoiWithHeader>(
    "region_of_interest", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the Image message.
  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic.c_str(), std::bind(&Detector::callback, this,  std::placeholders::_1),custom_qos_profile);

  // Initialize clock to timestamp the output messages
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}

Detector::~Detector(){}

void Detector::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
  detectAndDisplay(msg, this->get_logger());
}

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
int Detector::encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

// Receive Image message, apply body or face detection and publish the ROI
void Detector::detectAndDisplay(const shared_ptr<sensor_msgs::msg::Image> msg, rclcpp::Logger logger)
{
  // RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  i++;
  // Convert to an OpenCV matrix by assigning the data.
  Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);
  Mat frame_gray;
  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );
  //-- Detect bodies or faces
  vector<Rect> det;
  cascade.detectMultiScale( frame_gray, det );

  // cout << "/* Number of objects detected  */" << det.size()<<'\n';
  // If detected
  if (!det.empty()){
    for ( size_t i = 0; i < det.size(); i++ )
    {
      rectangle( frame, Point((det[i].x), det[i].y),Point((det[i].x + det[i].width-1), (det[i].y + det[i].height-1)),
         Scalar(255, 0, 0), 3, 8, 0);
      msg_out.roi.height = det[0].height;
      msg_out.roi.width = det[0].width;
      msg_out.roi.x_offset = det[0].x;
      msg_out.roi.y_offset = det[0].y;
      msg_out.roi.do_rectify = true;
    }
  }
  else{
    msg_out.roi.height = 0;
    msg_out.roi.width = 0;
    msg_out.roi.x_offset = 0;
    msg_out.roi.y_offset = 0;
    msg_out.roi.do_rectify = false;
  }
  // Calculate message loss
  if (i == 1){
    current_msg = std::stoi (msg->header.frame_id,nullptr,10);
  }
  else{
    msg_loss += std::stoi (msg->header.frame_id,nullptr,10) - current_msg - 1;
    current_msg = std::stoi (msg->header.frame_id,nullptr,10);
  }
  RCLCPP_INFO(logger, "Message loss %d", msg_loss);

  if (show_camera) {
    CvMat cvframe;
    if (msg->encoding == "rgb8") {
      Mat frame2;
      cvtColor(frame, frame2, COLOR_RGB2BGR);
      cvframe = frame2;
    } else {
      cvframe = frame;
    }
    cvShowImage("detection", &cvframe);
    // Draw the screen and wait for 1 millisecond.
    waitKey(1);
  }
  // rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  msg_out.header.frame_id = std::to_string(i);
  msg_out.header.stamp = clock->now();
  pub->publish(msg_out);
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  bool show_camera = true;
  bool body = false;
  std::string topic("image");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure parameters with command line options.
  // todo add height width
  if (!parse_command_options(
      argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &body, &topic))
  {
    return 0;
  }

  // Set the parameters of the quality of service profile. Initialize as the default profile
  // and set the QoS parameters specified on the command line.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = depth;
  custom_qos_profile.reliability = reliability_policy;
  custom_qos_profile.history = history_policy;

  std::cout << custom_qos_profile.reliability << '\n';
  // Create node and spin
  rclcpp::spin(std::make_shared<Detector>(topic, custom_qos_profile, show_camera, body));

  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();

  return 0;
}
