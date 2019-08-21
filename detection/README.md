# Contents
  * detection: Initializes a ROS 2 node which subscribes to an /image topic and performs face or body detection on received image. The node then publishes the  nettools_msgs/RoiWithHeader.msg on topic region_of_interest.
  * unified_target_publisher: Initializes a ROS 2 node (target_publisher) which subscribes to a region of interest topic and performs position estimation of detected face using the position of the camera as the reference point. Using the tf2_ros library the estimated position is transformed from the "external_camera" frame to the "map" frame and a PoseStamped message is published on "move_base_simple/goal" topic.  

# Prerequisites
1. ROS2 bouncy or crystal

# Build instructions
Clone in a new workspace e.g ros2_overlay_ws/src.

```
cd ros2_overlay_ws
colcon build --symlink-install
```
# Execute instructions
Make sure you have installed ros2 bouncy and setup the environment correctly.
Source the ros2_overlay_ws/install/local_setup.bash file.

## Detection

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

<p align="center">
    <img src="https://github.com/pantphot/thesis/blob/crystal/img/detection.png">
</p>

* The detection node can detect more than one objects but publishes the region of interest only for one.<br />

## Target publishing
In terminal D run:
```
cd ./path_to_ros2_ws
launch ./src/detection/launch/stfpub_targetpub.launch.py
```
This instruction executes unified_target_publisher and a static_transform_publisher, which publishes the position of the external camera in regards to the map.

In terminal E run:
```
cd ./path_to_ros2_ws
ros2 run map_server map_server ./src/maps/karto/karto_vergina.yaml
```
In terminal F run:
```
ros2 topic echo /move_base_simple/goal
```
to view the target sent to the robot or
```
ros2 topic echo /actual_target
```
to view the coordinates of the face which is detected.
