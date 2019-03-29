/*
*   position_estimation.cpp
*   Author: Pantelis Photiou
*   Created: Mar 2019
*   Initializes a ROS 2 node which subscribes to a region of interest topic
*   and performs position estimation of detected face .
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "position_estimation.hpp"
#include <cmath>
// #include "rclcpp/clock.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp/time_source.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "options.hpp"
// #include "detection.hpp"
//
// using namespace std;
// using namespace cv;
//
//
Position_Estimator::Position_Estimator (rmw_qos_profile_t custom_qos_profile)
: Node ("position_estimator"),
custom_qos_profile(custom_qos_profile),
image_width(),
face_height(16.0),
focal_length(),
hfov(),
thita(),
distance(),
displacement(),
phi(),
x(),
z()

{
  //   // Initialize publisher that publishes the ROI
  //   pub = this->create_publisher<nettools_msgs::msg::RoiWithHeader>(
  //     "region_of_interest", rmw_qos_profile_default);
  //
  // Initialize a subscriber that will receive the Image message.
  std::cerr << "Subscribing to topic '/region_of_interest'" << std::endl;
  sub = this->create_subscription<nettools_msgs::msg::RoiWithHeader>(
    "region_of_interest", std::bind(&Position_Estimator::callback, this,  std::placeholders::_1),custom_qos_profile);

    //   // Initialize clock to timestamp the output messages
    //   clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    //   std::cout << custom_qos_profile.depth << '\n';
  }
  //
  Position_Estimator::~Position_Estimator(){}
  //
  // Callback receiving the region of interest
  void Position_Estimator::callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg)
  {
    estimate(msg, this->get_logger());
  }

  // Receive region_of_interest and estimate person position
  void Position_Estimator::estimate(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger)
  {
    RCLCPP_INFO(logger, "Received message #%s", msg->header.frame_id.c_str());
    // Calculations based on camera reference point (0,0)

    // x_radians_per_pixel = 48.5/57.0/image_width
    image_width = msg->image_width;
    face_height = 16.0; //cm

    if (image_width == 352)
    {
      focal_length = 427.5; // measured in pixels
      hfov = 48.5 / 57.3; //radians
    }
    else
    {
      focal_length = 676.5;
      hfov = 51.2 / 57.3;
    }
    thita = hfov / 2; //radians

    // Distance = Height_in_cm_of_face * focal_length / Heigth_in_pixels
    distance = face_height * focal_length / msg->roi.height;//cm
    // distance from the center of the image in pixels
    displacement =  msg->roi.x_offset + msg->roi.width / 2 - image_width/2;
    // angle from the center of the image in radians
    phi = atan   ((2 * displacement * tan(thita) / image_width)) ;

    x = distance * sin(phi);//cm
    z = distance * cos(phi);


    // RCLCPP_INFO(logger, "imagewidth = %d ", image_width);
    RCLCPP_INFO(logger, "distance = %lf ", distance);
    RCLCPP_INFO(logger, "phi = %f deg", phi*57.3);
    RCLCPP_INFO(logger, "x = %f cm" ,x);
    RCLCPP_INFO(logger, "z = %f cm", z);


    // std_msgs/Header header       # Two-integer timestamp that is expressed as seconds and nanoseconds. builtin_interfaces/Time stamp
    //                              # sequence number. string frame_id
    // sensor_msgs/RegionOfInterest roi #uint32 x_offset  Leftmost pixel of the ROI
    //                                  #uint32 y_offset  Topmost pixel of the ROI
    //                                  #uint32 height    # Height of ROI
    //                                  #uint32 width     # Width of ROI
    //
  }




  int main(int argc, char * argv[])
  {
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure parameters with command line options.
    // todo add height width

    // Set the parameters of the quality of service profile. Initialize as the default profile
    // and set the QoS parameters specified on the command line.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    // custom_qos_profile.depth = depth;
    // custom_qos_profile.reliability = reliability_policy;
    // custom_qos_profile.history = history_policy;


    // Create node and spin
    rclcpp::spin(std::make_shared<Position_Estimator>(custom_qos_profile));

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
  }
