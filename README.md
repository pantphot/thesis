# Design and implementation of a hybrid system to satisfy execution time constraints for software built with ROS1, ROS2 and IoT frameworks

This diploma thesis (https://drive.google.com/open?id=1j1NYYV4U7ih4Pj1HSqtCprCo9z_a6Ghv) presents the development of an IoT application, which allows communication between smart devices and robots. For the creation of the system, the use of the ROS2 framework was explored. ROS2 is the latest version of the Robot Operating System (ROS), the most used robotic framework in our day, which uses various implementations of the DDS (Data Distribution Service) communication protocol. DDS is a real-time, data-centric, publish-subscribe protocol created specifically to meet the needs of a fully distributed IoT system.
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
* MongoDB
* FIWARE IoT platform
* Node-RED

## Methodology
* Creation of a ROS2 tool (nettools package) for the calculation and visualization of topic statistics (latency, frequency, throughput, message loss, packet delay variation)
* Configuring the QoS parameters and studying network behavior
* Creation of a ROS2 tool (detection package) responsible for face detection in image messages and calculation of its location on given map
* TurtleBot2 navigation system using ROS 1
* Choosing the most suitable architecture and combination of QoS policies to
meet the requirements of the system / application.

## Application architecture
![Application](https://github.com/pantphot/thesis/blob/crystal/img/appA.jpeg)
* Application extension 1: Connection of 2 different DDS domains using eProsima Integration Service
![Application](https://github.com/pantphot/thesis/blob/crystal/img/extension1.jpeg)
* Application extension 2: Connection of the system to FIWARE platform, using FIROS2 for sending alert messages to remote user through Node-RED dashboard  
![Application](https://github.com/pantphot/thesis/blob/crystal/img/extension2.jpeg)

* The Node-RED flow created for sending periodic HTTP requests to FIWARE's Orion Broker and receiving latest intruder coordinates.

<p align="center">
    <img src="https://github.com/pantphot/thesis/blob/crystal/img/Node-RED_flow.png">
</p>

* The Node-RED dashboard created for delivering alerts to remote user

<p align="center">
    <img src="https://github.com/pantphot/thesis/blob/crystal/img/Node-RED_dashboard.png">
</p>

## Demo
<img align="center" width="900" height="562.5" src="https://github.com/pantphot/thesis/blob/crystal/img/demo.gif">
