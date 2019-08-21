# Build instructions
For each example create a build directory and use cmake and make to create the necessary binary files e.g.:

```
cd point_to_fiware
mkdir build
cd build
cmake ..
make
```

# Build instructions
For each example create a build directory and use cmake and make to create the necessary binary files e.g.:

```
cd point_to_fiware
mkdir build
cd build
cmake ..
make
```
# Execute Prerequisites
1. FIWARE image up and running
2. FIROS2 sourced in environment

# Usage
First a FIWARE image must be up and running.
Then a NGSIv2 entity must be created:
```
cd ros2_workspace
./firos2_examples/point_to_fiware/build/cb_create_point
```

Executing
```
firos2 ./point_to_fiware/config_pub.xml
```
creates a firos2 connector between a ROS2 subscriber on topic /detected_point and a NGSIv2 publisher, that sends the entity's data to FIWARE's Orion Broker.
