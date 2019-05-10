/*
*   unified_target_publisher.cpp
*   Author: Pantelis Photiou
*   Created: May 2019
*   Initializes a ROS 2 node which subscribes to a region of interest topic
*   and performs position estimation of detected face using the position of the
*   camera as the reference point. The node then transforms the PoseStamped to map frame and publishes on
*   "move_base_simple/goal" topic.
*/

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "unified_target_publisher.hpp"
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "options.hpp"


Unified_Target_Publisher::Unified_Target_Publisher (rmw_qos_profile_t custom_qos_profile)
: Node ("target_publisher"),
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
y(),
tfBuffer(clock),
tf2_listener(tfBuffer)

{
  //   // Initialize publisher that publishes the pose
  pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "move_base_simple/goal", rmw_qos_profile_default);

  // Initialize a subscriber that will receive the Image message.
  std::cerr << "Subscribing to topic '/region_of_interest'" << std::endl;
  sub = this->create_subscription<nettools_msgs::msg::RoiWithHeader>(
    "region_of_interest", std::bind(&Unified_Target_Publisher::callback, this,  std::placeholders::_1),custom_qos_profile);


      // Initialize clock to timestamp the output messages
  // clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tf2_listener;

  // tf2_ros::Buffer tfBuffer(Unified_Target_Publisher.get_clock());
  // tf2_ros::Buffer tfBuffer(clock);
  // tfBuffer(*clock,dur,false);
  // tf2_ros::TransformListener tf2_listener(tfBuffer);
  }
  //
  Unified_Target_Publisher::~Unified_Target_Publisher(){}
  //
  // Callback receiving the region of interest
  void Unified_Target_Publisher::callback(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg)
  {
    estimate(msg, this->get_logger());
  }

  // Receive region_of_interest and estimate person position
  void Unified_Target_Publisher::estimate(const std::shared_ptr<nettools_msgs::msg::RoiWithHeader> msg, rclcpp::Logger logger)
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


    RCLCPP_INFO(logger, "distance = %lf ", distance);

    msg_out.header.frame_id = "external_camera";
    msg_out.header.stamp = clock -> now();
    msg_out.pose.position.y = -x/100;
    msg_out.pose.position.z = y/100;
    msg_out.pose.position.x = z/100;
    msg_out.pose.orientation.x = 0.0;
    msg_out.pose.orientation.y = 0.0;
    msg_out.pose.orientation.z = 0.0;
    msg_out.pose.orientation.w = 1.0;

    try{
    	tfBuffer.transform(msg_out,msg_out,target_fr);
      	if (msg_out.pose.position.x < 10000.0){
      		msg_out.pose.position.z=0;
    		RCLCPP_INFO(logger,"Target Coordinates = (%lf , %lf)",msg_out.pose.position.x,msg_out.pose.position.y);
		    msg_out.header.stamp = clock -> now();
    		pub -> publish(msg_out);
   	 }
    }
    catch (tf2::LookupException e)
    {std::cout << e.what() << '\n';}

  }

  int main(int argc, char * argv[])
  {
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    bool show_camera = false;
    bool body = false;
    std::string topic("region_of_interest");
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure parameters with command line options.
    if (!parse_command_options(
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &body, &topic))
    {
      return 0;
    }

    // Set the parameters of the quality of service profile. Initialize as the default profile.

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = depth;
    custom_qos_profile.reliability = reliability_policy;
    custom_qos_profile.history = history_policy;
    // Create node and spin
    rclcpp::spin(std::make_shared<Unified_Target_Publisher>(custom_qos_profile));

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
  }
