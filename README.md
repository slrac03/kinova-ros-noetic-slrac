# Kinova ROS Noetic Integration

This repository contains a ROS package for integrating Kinova robotic arms with ROS Melodic. It provides drivers, launch files, and example nodes for controlling Kinova arms using ROS topics and services.

## Installation
### ROS
1. Make sure you have [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) v1.17.0 installed.
2. Also use [catkin tools](https://catkin--tools-readthedocs-io.translate.goog/en/latest/history.html?_x_tr_sl=en&_x_tr_tl=es&_x_tr_hl=es&_x_tr_pto=tc)
   ```bash
   sudo apt install python3-catkin-tools
3. Install [ros_bridge_server](https://wiki.ros.org/rosbridge_server)

   ```bash
   sudo apt install ros-noetic-rosbridge-suite

4. Clone this repository into your `catkin_ws` folder:

   ```bash
   cd ~/catkin_ws
   git clone https://github.com/slrac03/kinova-ros-noetic-slrac.git

Make sure to remove the old build artifacts with:

```bash
rm -rf build devel logs
```

Then rebuild with:

```bash
catkin build
```

Make sure to change the IP address to 192.168.1.X, knowing that the robot Addr is 192.168.1.10

if done correctly:
```bash
roslaunch file_server ros_sharp_communication.launch 
```
   

### Build kortex

ROS KINOVA KORTEX™ is the official ROS package to interact with KINOVA KORTEX™ and its related products. It is built upon the KINOVA KORTEX™ API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

These are the instructions to run in a terminal to create the workspace, clone the [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repository and install the necessary ROS dependencies:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan==1.59
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        rosdep install --from-paths src --ignore-src -y

> Instructions are for conan V1.X only and it won't work for versions >=2.0.0

Then, to build and source the workspace:

        catkin_make
        source devel/setup.bash


# Build and Run instructions

[Python API](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/examples/readme.md) 
