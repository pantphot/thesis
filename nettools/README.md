# Contents
  * latency: Creates a ROS2 subscriber to given TOPIC and calculates network statistics (message loss, latency, frequency,interarrival jitter), which then is published on the topic_statistics_TOPIC topic.
  * throughput:Creates a ROS2 to given TOPIC and calculates network throughput , which then is published on the topic_statistics_TOPIC topic.
  * variable_length_pub:  Publishes variable length messages with given increase step and frequency.
  * variable_length_pub.py:  Publishes variable length messages with given increase step and frequency. Only works with ROS2 crystal
  * nettools_plotter.py: A script for plotting the calculated statistics using Python matplotlib library.   

# Prerequisites
1. ROS2 bouncy or crystal
2. python3
3. matplotlib
4. nettools_msgs 

# Build instructions
Clone in a new workspace e.g ros2_overlay_ws/src.

```
cd ros2_overlay_ws
colcon build --symlink-install
```
# Execute instructions
Make sure you have installed ros2 crystal and setup the environment correctly.
Source the ros2_overlay_ws/install/local_setup.bash file.

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
<p align="center">
    <img src="https://github.com/pantphot/thesis/blob/crystal/img/nettools_plotter.gif">
</p>

* To be able to 'ros2 run' the nettools_plotter.py you have to make it executable before building the package (chmod +x nettools_plotter.py). <br />
* The nettools latency node subscribes to topics where there is a std_msgs/header included in the message published. To add more topic types add new else if in latency.cpp main() (line 155) and include the message definition in the includes section.  <br />
* (nettools throughput) To add more topic types add new else if with new subscription creation in throughput.cpp, in throughput class initialization (line 41) and include the message definition in the includes section.  <br />
