# Contents
1. image_tools demo: (https://github.com/ros2/demos/tree/master/image_tools/include/image_tools) with modified cam2image.cpp file , to include timestamp in published image.
2. nettools_msgs: custom message interface including network statistics calculated by the  by the nettools package.
3. nettools:
    * latency.cpp: Subscribes to given TOPIC and calculates network statistics (message loss, latency, frequency,interarrival jitter), which then is published on the topic_statistics_TOPIC topic.
    * throughput.cpp: Subscribes to given TOPIC and calculates network throughput , which then is published on the topic_statistics_TOPIC topic.
    * variable_length_pub.cpp:  Publishes variable length messages with given increase step and frequency.
    * variable_length_pub.py:  Publishes variable length messages with given increase step and frequency. Only works with ROS2 crystal
    * nettools_plotter.py: A script for plotting the calculated statistics using Python matplotlib library .   
4. detection:
    * detection.cpp: Initializes a ROS 2 node which subscribes to an image topic and performs face or body detection on received image. The node then publishes the region of interest on topic region_of_interest.

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
to view if something was detected and the region of interest offsets from the original image


* To be able to run the nettools_plotter.py with ros2 run you have to make it executable before building the package (chmod +x nettools_plotter.py). <br />
* The nettools latency node subscribes to topics where there is a header included in the message published. To add more topic types add new else if in latency.cpp main() (line 155) and include the message definition in the includes section.  <br />
* (nettools throughput) To add more topic types add new else if with new subscription creation in throughput.cpp, in throughput class initialization (line 41) and include the message definition in the includes section.  <br />
* The detection package can detect more than one objects but publishes the region of interest only for one.<br />
