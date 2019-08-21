# Contents
 Contains custom ROS2 messages used by nettools and detection packages
 * StatisticsMeasurments.msg and TopicStatistics.msg include network statistics calculated by the  by the nettools package.
 * RoiWithHeader.msg, RoiWithHeaderCompact and PointHeader.msg are modified versions of ROS2 common interfaces RegionOfInterest.msg and Point.msg, including headers, so it is possible to calculate topic statistics with the nettools package nodes. They are also translated by the FIROS2 node to NGSIv2 format so they can be stored in the FIWARE database.

# Prerequisites
1. ROS2 bouncy or crystal
2. FIROS2

# Build instructions
Clone in a new workspace e.g ros2_overlay_ws/src. Make sure you have installed ros2 bouncy/crystal and setup the environment correctly.
Source the ros2_overlay_ws/install/local_setup.bash file. Also source the FIROS2 package local_setup.bash file.

```
cd ros2_overlay_ws
colcon build --symlink-install
```
