# Design and implementation of a hybrid system to satisfy execution time constraints for software built with ROS1, ROS2 and IoT frameworks

This diploma thesis (https://drive.google.com/open?id=1j1NYYV4U7ih4Pj1HSqtCprCo9z_a6Ghv) presents the development of an IoT application, which allows communication between smart devices and robots. For the creation of the system, the use of the ROS2 framework was explored. ROS2 is the latest version of the Robot Operating System (ROS), the most used robotic framework in our day, which implements / uses the DDS (Data Distribution Service) communication protocol. DDS is a real-time, data-centric, publish-subscribe protocol created specifically to meet the needs of a fully distributed IoT system.
The basic function of the implemented application is the collaboration of three devices for the observation of a space and the detection of intruders. The devices used are:
* Raspberry Pi (Raspian Stretch): Detects faces in video stream.
* Intel NUC7i3BNH (Ubuntu 18.04): Used on the TurtleBot2 for the robot navigation. Orbbec Astra Depth Camera is used for the robot localization.  
* Computer (Ubuntu 18.04): Used for supporting the application and visualization of various parameters

## Tools and technologies used
* ROS
* ROS2
* eProsima FastRTPS
* eProsima Integration Service
* eProsima FIROS2
* Docker Container
* FIWARE IoT platform
* Node-RED

## Methodology
* Creation of a ROS2 tool (nettools package) for the calculation and visualization of topic statistics (latency, frequency, throughput, message loss, packet delay variation)
* Configuring the QoS parameters and studying network behavior
* Creation of a ROS2 tool (detection package) responsible for face detection in image messages and calculation of its location on given map
* TurtleBot2 navigation system using ROS 1
* Choosing the most suitable architecture and combination of QoS policies to
meet the requirements of the system / application.
![Application](https://github.com/pantphot/thesis/blob/crystal/img/appA.jpeg)
* Application extension 1: Connect 2 different


*
1. image_tools demo: (https://github.com/ros2/demos/tree/master/image_tools/include/image_tools) Modified cam2image.cpp file to include timestamp in published image and calculate average publishing frequency.
2. nettools_msgs: Contains custom messages including network statistics calculated by the  by the nettools package and ROS2 region of interest message including header.
3. nettools:
    * latency: Creates a ROS2 subscriber to given TOPIC and calculates network statistics (message loss, latency, frequency,interarrival jitter), which then is published on the topic_statistics_TOPIC topic.
    * throughput:Creates a ROS2 to given TOPIC and calculates network throughput , which then is published on the topic_statistics_TOPIC topic.
    * variable_length_pub:  Publishes variable length messages with given increase step and frequency.
    * variable_length_pub.py:  Publishes variable length messages with given increase step and frequency. Only works with ROS2 crystal
    * nettools_plotter.py: A script for plotting the calculated statistics using Python matplotlib library.   
4. detection:
    * detection: Initializes a ROS 2 node which subscribes to an /image topic and performs face or body detection on received image. The node then publishes the  nettools_msgs/RoiWithHeader.msg on topic region_of_interest.
    * unified_target_publisher: Initializes a ROS 2 node (target_publisher) which subscribes to a region of interest topic and performs position estimation of detected face using the position of the camera as the reference point. Using the tf2_ros library the estimated position is transformed from the "external_camera" frame to the "map" frame and a PoseStamped message is published on "move_base_simple/goal" topic.  
5. fiware: Contains a docker-compose.yml file for creating a MongoDB instance and a FIWARE Orion Broker.
6. domain_change: Contains xml files used by eProsima Integration Service (https://github.com/eProsima/Integration-Service) for connecting DDS participants from different DDS domains.   
7. maps: Contains the map files which are used by the navigation stack for the Turtlebot navigation in the laboratory.

# Prerequisites
1. ROS2 bouncy or crystal
2. python3
3. matplotlib

# Build instructions
Clone in a new workspace e.g ros2_overlay_ws/src.

```
cd ros2_overlay_ws
colcon build --symlink-install
```
# Execute instructions
Make sure you have installed ros2 bouncy and setup the environment correctly.
Source the ros2_overlay_ws/install/local_setup.bash file.

## nettools package

In terminal A run the image publisher:
```
ros2 run image_tools cam2image
```
or:
```
ros2 run nettools variable_length_pub
```
for command line arguments list run with -h flag

In terminal B run the nettools package.
For throughput statistics run:
```
ros2 run nettools throughput
```
for command line arguments list run with -h flag

For the rest of the statistics run:
```
ros2 run nettools latency
```
for command line arguments list run with -h flag

To view the calculated statistics run:
```
ros2 topic echo /topic_statistics_TOPIC
```

To view a real-time plot for a given statistic run the nettools_plotter.py script followed by the statistic you want to plot e.g.:
```
ros2 run nettools nettools_plotter.py latency
```
## detection package
In terminal A run:
```
cd ./path_to_ros2_ws
ros2 run detection detection -s 1
```
In terminal B run:
```
ros2 run image_tools cam2image
```
for command line arguments list run with -h flag

In terminal C run:
```
ros2 topic echo /region_of_interest
```
to view if something was detected and the region of interest offsets from the original image.



* To be able to 'ros2 run' the nettools_plotter.py you have to make it executable before building the package (chmod +x nettools_plotter.py). <br />
* The nettools latency node subscribes to topics where there is a std_msgs/header included in the message published. To add more topic types add new else if in latency.cpp main() (line 155) and include the message definition in the includes section.  <br />
* (nettools throughput) To add more topic types add new else if with new subscription creation in throughput.cpp, in throughput class initialization (line 41) and include the message definition in the includes section.  <br />
* The detection package can detect more than one objects but publishes the region of interest only for one.<br />
