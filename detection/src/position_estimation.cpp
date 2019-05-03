/*
*   position_estimation.cpp
*   Author: Pantelis Photiou
*   Created: Mar 2019
*   Initializes a ROS 2 node which subscribes to a region of interest topic
*   and performs position estimation of detected face using the position of the
*   camera as the reference point. Publishes on "detection_pose" topic.
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "position_estimation.hpp"
#include <cmath>

Position_Estimator::Position_Estimator (rmw_qos_profile_t custom_qos_profile)
: Node ("position_estimator"),
custom_qos_profile(custom_qos_profile),
image_width(),
image_height(),
face_height(16.0),
focal_length(),
hfov(),
vfov(),
thita_x(),
thita_y(),
distance(),
displacement_x(),
displacement_y(),
phi_x(),
phi_y(),
x(),
z(),
y()

{
  //   // Initialize publisher that publishes the pose
  pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "detection_point", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the Image message.
  std::cerr << "Subscribing to topic '/region_of_interest'" << std::endl;
  sub = this->create_subscription<nettools_msgs::msg::RoiWithHeader>(
    "region_of_interest", std::bind(&Position_Estimator::callback, this,  std::placeholders::_1),custom_qos_profile);


      // Initialize clock to timestamp the output messages
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

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
    image_width = msg->image_width;
    image_height = msg->image_height;
    face_height = 16.0; //cm

    if (image_width == 352)
    {
      focal_length = 427.5; // measured in pixels
      hfov = 48.5 ; //radians
      vfov = 38.1;
    }
    else
    {
      focal_length = 676.5;
      hfov = 51.2;
      vfov = 39.82;
    }
    thita_x = hfov / 2 /57.3; //radians
    thita_y = vfov / 2 /57.3; //radians

    // Distance = Height_in_cm_of_face * focal_length / Heigth_in_pixels
    distance = face_height * focal_length / float(msg->roi.height);//cm
    // distance from the center of the image in pixels
    displacement_x = float(msg->roi.x_offset) + float(msg->roi.width) / 2 - float(image_width/2);
    displacement_y = float(image_height/2) - (float(msg->roi.y_offset) + float(msg->roi.height) / 2);
    // angle from the center of the image in radians
    phi_x = atan   ((2 * displacement_x * tan(thita_x) / image_width)) ;
    phi_y = atan   ((2 * displacement_y * tan(thita_y) / image_height)) ;


    x = distance * sin(phi_x);//cm
    z = distance * cos(phi_x);
    y = distance * sin(phi_y);

    // // RCLCPP_INFO(logger, "imagewidth = %d ", image_width);
    // RCLCPP_INFO(logger, "image_height = %d ", image_height);
    // RCLCPP_INFO(logger, "msg->roi.y_offset = %d ", msg->roi.y_offset);
    // RCLCPP_INFO(logger, "msg->roi.heigth/2 = %d ",msg->roi.height/2);
    // RCLCPP_INFO(logger, "image_height/2 = %d ",image_height/2);
    // RCLCPP_INFO(logger, "displacement_x  = %lf ", displacement_x);
    // RCLCPP_INFO(logger, "displacement_y  = %lf ", displacement_y);
    RCLCPP_INFO(logger, "distance = %lf ", distance);
    // RCLCPP_INFO(logger, "phi_x = %f deg", phi_x*57.3);
    // RCLCPP_INFO(logger, "phi_y = %f deg", phi_y*57.3);
    RCLCPP_INFO(logger, "x = %f cm" ,x);
    RCLCPP_INFO(logger, "y = %f cm", y);
    // RCLCPP_INFO(logger, "z_y = %f cm", z_y);
    RCLCPP_INFO(logger, "z_x = %f cm", z);

    msg_out.header.frame_id = "external_camera";
    msg_out.header.stamp = clock -> now();
    msg_out.point.y = -x/100;
    msg_out.point.z = y/100;
    msg_out.point.x = z/100;
    // msg_out.pose.position.y = -x/100;
    // msg_out.pose.position.z = y/100;
    // msg_out.pose.position.x = z/100;
    // msg_out.pose.orientation.x = 0.0;
    // msg_out.pose.orientation.y = 0.0;
    // msg_out.pose.orientation.z = 0.0;
    // msg_out.pose.orientation.w = 1.0;

    pub -> publish(msg_out);

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

    // Set the parameters of the quality of service profile. Initialize as the default profile.

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // Create node and spin
    rclcpp::spin(std::make_shared<Position_Estimator>(custom_qos_profile));

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
  }
