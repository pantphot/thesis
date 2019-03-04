#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

CascadeClassifier face_cascade;

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}


/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
  // copy cv information into ros message
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

/// Detect face on an OpenCV matrix
/**
 * \param[in] frame The OpenCV matrix.
 * \param[out] frame including bounding box around detected face.
 */

void detectAndDisplay( Mat frame )
{
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
        Mat faceROI = frame_gray( faces[i] );
    }
    //-- Show what you got
    imshow( "Capture - Face detection", frame );
}


int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  bool show_camera = false;
  size_t depth = 10;
  double freq = 30.0;
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  size_t width = 320;
  size_t height = 240;
  bool burger_mode = true;
  std::string topic("image");

  // Load the cascades
  auto face_cascade_name = "./src/detection/data/haarcascade_frontalface_alt.xml";
  if( !face_cascade.load( face_cascade_name ) )
  {
      cout << "--(!)Error loading face cascade\n";
  };
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure demo parameters with command line options.
  // bool success = parse_command_options(
  //   argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &freq, &width, &height,
  //   &burger_mode, &topic);
  // if (!success) {
  //   return 0;
  // }
    // CommandLineParser parser(argc, argv,
    //                          "{help h||}"
    //                          // "{face_cascade|../include/data/haarcascades/haarcascade_frontalface_alt.xml|Path to face cascade.}"
    //                          // "{eyes_cascade|../include/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|Path to eyes cascade.}"
    //                          "{camera|0|Camera device number.}");
    // parser.about( "\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
    //               "You can use Haar or LBP features.\n\n" );
    // parser.printMessage();
    // String face_cascade_name = parser.get<String>("face_cascade");
    // String eyes_cascade_name = parser.get<String>("eyes_cascade");
    // auto eyes_cascade_name = "./src/detection/data/haarcascade_eye_tree_eyeglasses.xml";

    // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
    auto node = rclcpp::Node::make_shared("face_detector");
    rclcpp::Logger node_logger = node->get_logger();

    // Set the parameters of the quality of service profile. Initialize as the default profile
    // and set the QoS parameters specified on the command line.
    rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;

    // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
    custom_camera_qos_profile.depth = depth;

    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    custom_camera_qos_profile.reliability = reliability_policy;

    // The history policy determines how messages are saved until the message is taken by the reader.
    // KEEP_ALL saves all messages until they are taken.
    // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    // parameter.
    custom_camera_qos_profile.history = history_policy;

    RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
    // Create the image publisher with our custom QoS profile.
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(
      topic, custom_camera_qos_profile);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(freq);

    cv::VideoCapture cap;

    // Initialize OpenCV video capture stream.
    // Always open device 0.
    cap.open(0);

    // Set the width and height based on command line arguments.
    cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap.isOpened()) {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      return 1;
    }

    // Initialize OpenCV image matrices.
    cv::Mat frame;

    // Initialize a shared pointer to an Image message.
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;

    size_t i = 1;

    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok()) {
      // Get the frame from the video capture.
      cap >> frame;

      // Check if the frame was grabbed correctly
      if (!frame.empty()) {

        // Apply the classifier to the frame
        detectAndDisplay( frame );
        // Convert to a ROS image
        convert_frame_to_message(frame, i, msg);
        if (show_camera) {

          CvMat cvframe = frame;
          // Show the image in a window called "detection".
          cvShowImage("detection", &cvframe);
          // Draw the image to the screen and wait 1 millisecond.
          cv::waitKey(1);
        }
        // Publish the image message and increment the frame_id.
        RCLCPP_INFO(node_logger, "Publishing image #%zd", i);


        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        msg->header.stamp = clock->now();


        pub->publish(msg);
        ++i;
      }
      // Do some work in rclcpp and wait for more to come in.
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    rclcpp::shutdown();

    // while ( capture.read(frame) )
    // {
    //     if( frame.empty() )
    //     {
    //         cout << "--(!) No captured frame -- Break!\n";
    //         break;
    //     }
    //
    //     if( waitKey(10) == 27 )
    //     {
    //         break; // escape
    //     }
    // }
    return 0;
}
