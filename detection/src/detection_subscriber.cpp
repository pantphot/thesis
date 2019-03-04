/*
*   detection_subscriber.cpp
*   Author: Pantelis Photiou
*   Created: Mar 2019
*   Initializes a ROS 2 node which subscribes to an image topic
*   and performs face detection on received image.
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
#include "options.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

CascadeClassifier face_cascade;

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.

int encoding2mat_type(const std::string & encoding)
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

/// Convert the ROS Image message to an OpenCV matrix and display it to the user.

void detectAndDisplay(const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());

  if (show_camera) {
    // Convert to an OpenCV matrix by assigning the data.
    Mat frame(
      msg->height, msg->width, encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces );

    // If detected
    // if (faces.empty()){
    //   std::cout << "/* Not detected */" << '\n';
    // }

    for ( size_t i = 0; i < faces.size(); i++ )
    {
      Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
      ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 0 ), 4 );
      Mat faceROI = frame_gray( faces[i] );
    }

    CvMat cvframe;
    if (msg->encoding == "rgb8") {
      Mat frame2;
      cvtColor(frame, frame2, COLOR_RGB2BGR);
      cvframe = frame2;
    } else {
      cvframe = frame;
    }

    cvShowImage("face_detector", &cvframe);
    // Draw the screen and wait for 1 millisecond.
    waitKey(1);
  }
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  bool show_camera = true;
  std::string topic("image");

  // Load the cascades
  auto face_cascade_name = "./src/detection/data/haarcascade_frontalface_alt.xml";
  if( !face_cascade.load( face_cascade_name ) )
  {
    cout << "Error loading face cascade!\n";
  };
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure demo parameters with command line options.
  if (!parse_command_options(
      argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, nullptr,
      nullptr,nullptr, &topic))
  {
    return 0;
  }

  if (show_camera) {
    std::cerr << "Creating window" << std::endl;
    // Initialize an OpenCV named window called "face_detector".
    cvNamedWindow("face_detector", CV_WINDOW_AUTOSIZE);
    waitKey(1);
    std::cerr << "After creating window" << std::endl;
  }

  // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
  auto node = rclcpp::Node::make_shared("face_detector");
  rclcpp::Logger node_logger = node->get_logger();

  // Set the parameters of the quality of service profile. Initialize as the default profile
  // and set the QoS parameters specified on the command line.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
  custom_qos_profile.depth = depth;

  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  custom_qos_profile.reliability = reliability_policy;

  // The history policy determines how messages are saved until the message is taken by the reader.
  // KEEP_ALL saves all messages until they are taken.
  // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
  // parameter.
  custom_qos_profile.history = history_policy;

  // Define a callback function for the subscription
  auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      detectAndDisplay(msg, show_camera, node->get_logger());
    };


  RCLCPP_INFO(node_logger, "Subscribing on topic '%s'", topic.c_str());

  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, callback, custom_qos_profile);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
