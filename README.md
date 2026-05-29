# UMRT ROS Template
ROS project repository template for the University of Manitoba Robotics Team.

New projects should be **forked** from this repo (not using this as a template, as that prevents template changes from
trickling down). Each new project must:
1. Fill in missing fields in package.in.xml
2. Fill in project name in CMakeLists.txt and Doxyfile
3. Go into `umrt-build` package settings and give the new repo read permission
4. Go into `umrt-apt-image` package settings and give the new repo read permission
5. Go into `UMRoboticsTeam` organisation secrets and add the new repo to:
   - `APT_DEPLOY_KEY`
   - `APT_SIGNING_KEY`
6. Copy the rulesets (branch protection rules) from a mature repository like
   [umrt-arm-firmware-lib](https://github.com/UMRoboticsTeam/umrt-arm-firmware-lib/)
7. Remove this notice and fill in below README template
8. Write something in mainpage.dox
9. Replace example files with real code, add source files to CMake targets, document it with Doxygen, and proceed

---
# Heading 

This project implements heading for the University of Manitoba Robotics Team's 
rover.

**Overview**
Dual GPS localization and heading calculation package for the University of Manitoba.
This package uses 2 SparkFun GNSS Combo Breakout to:
   1.   Get rover position
   2.   Compute rover heading
   3.   Publish a combined GPS topic.
This combo breakout pairs the u-blox ZED-F9P multi-band high-precision GNSS module with the NEO-D9S L-band GNSS correction data receiver. In this setup, only ZED-F9P is used for position and heading estimation. Website to find the details about this board: datasheet. 

**Hardware Setup**
Both GPS receivers are connected to Jetson through the USB-A Ports. Each receiver appears as its own serial USB device inside Linux.
Each GPS receiver is connected to its own external GNSS antenna. The antennas receive satellite signals while the receivers process the GNSS data and publish GPS coordinates.
The antennas are mounted left and right on the rover, connected to their respective GPS receivers. This creates a horizontal baseline and the package uses this setup to estimate heading.

**Header Files and Heading calculation**
The header file, gps_node.hpp defines:
* Publishers: Sends messages to a topic. Other nodes can subscribe to it.
* Subscribers: Listens to a topic. Receives messages when published.
* Some variables and functions: Store and process the GPS data.

The heading_node.cpp file uses the header file to:
* Initialize publisher, subscriber, variables and functions
* Wait until the GPS actually receives data to start processing it.
* Use the latitude and longitude positions of both GPS to calculate the heading.

**Heading Estimation**
First, use the latitude and longitude coordinates to calculate the difference between left and right GPS in meters. Since the gps is mounted on the left and right of the rover, its forward is perpendicular to the baseline. Thus, we rotate the vector 90° to make the rover point forward:
double dx = (gpsRight_msg.longitude - gpsLeft_msg.longitude) * cos(((gpsRight_msg.latitude + gpsLeft_msg.latitude) / 2.0) * M_PI / 180.0) * 111320.0;
double dy = (gpsRight_msg.latitude - gpsLeft_msg.latitude) * 111320.0;
double forward_x = -dy;
double forward_y = dx;

Now, to get the heading angle, just use atan2(x , y) to convert the forward vector into an angle. 
heading_angle = atan2(forward_x, forward_y) * 180.0 / M_PI;

Then, compare this angle to the angle from a compass to get the heading direction. Finally, publish all this data to the topic.

**Node**
The *gps_node* created in the heading_node.cpp file, is the only node created in this package to take in the coordinates from the 2 GPS receivers and publish an output.

**Topics**
There are 3 main topics:
* /gps_left/fix : This collects and processes the GPS coordinates from the left GPS.
* /gps_right/fix : This collects and processes the GPS coordinates from the right GPS.
* gps/fix : This publishes the combined GPS data.
  
Steps to run the package:
| Terminal Steps | Details | Example Command |
|---|---|---|
|  | Go inside the directory | `cd Documents/GPS` |
|  | Build the Image file (You need to do this every time you make changes to your files) | `./gpsImage.sh` |
|  | Run the container with the name ‘gps’ | `./gpsContainer.sh gpsImage` |
|  | (Every time) source the file | `source install/setup.bash` |
|  | Launch the rosbridge server | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
|  | (New terminal) Connect the container to this new terminal using: | `docker exec -it gpsImage bash` |
|  | Run the launch file | `ros2 launch umrt-localization-ros gps.launch.py` |
|  | (New terminal) Connect the container to this new terminal using: | `docker exec -it gpsImage bash` |
|  | Run the node | `ros2 run umrt-localization-ros gps_node` |
|  | (New Terminal) Connect one terminal’s container to another terminal | `ros2 topic echo /gps/fix` |
| **In Foxglove** | Run the topic | `ros2 topic echo /gps/fix` |
|  | Go to Dashboard |  |
|  | Click on open connection |  |
|  | On the pop-up window, select rosbridge |  |
|  | Click on open |  |
|  | Click on the dot, and then on the taskbar on the left select topic, and then choose /fix |  |

**Overview on Launch File:**
The launch file distinguishes the 2 gps via their serial string, which we were able to modify with ublox’s software U-center.

Once the launch file is ran, our 2 ublox dgnss nodes are created, and they will then output a nav-sat fix, which the heading node (gps_node) will use to calculate a bearing. Below is a diagram to illustrate the process.
<img width="851" height="521" alt="finalfinalfinal drawio" src="https://github.com/user-attachments/assets/39a0ff3c-babb-465c-b657-6ee1f31d4a47" />


[See the documentation](https://umroboticsteam.github.io/********** project-name **********/)
