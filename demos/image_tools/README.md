# Contents
image_tools demo: (https://github.com/ros2/demos/tree/master/image_tools/include/image_tools) Modified cam2image.cpp file to include timestamp in published image and calculate average publishing frequency.

# Prerequisites
1. ROS2 bouncy or crystal

# Build instructions
Clone in a new workspace e.g ros2_overlay_ws/src.

```
cd ros2_overlay_ws
colcon build --symlink-install
```
# Execute instructions
Make sure you have installed ros2 bouncy/crystal and setup the environment correctly.
Source the ros2_overlay_ws/install/local_setup.bash file.

In a terminal A run the image publisher:
```
ros2 run image_tools cam2image
```
for command line arguments list run with -h flag
