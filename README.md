# ros2_tutorial

## Installation

git clone this package into the /src under any of your ros2  workpsace directory. If your ros2 workspace is named as "ros2_ws" under your home directory,

```
$ cd ~/ros2_ws/src
$ git clone https://github.com/kyuhyong/ros2_tutorial.git
$ cd ..
$ colcon build --symlink-install
```
Once colcon build is done for the first time, make sure to source setup.bash under /install directory.
```
source install/setup.bash
```

## tutorial_rclcpp_pub_sub

**tutorial_rclcpp_pub_sub** constains executable **rclcpp_pub_sub**.
Also includes
- Custom messages in /msg folder

## tutorial_rclcpp_service

**tutorial_rclcpp_service** package contains executable **rclcpp_service_server** 
which includes 
- Custom services in /srv folder

and create a server to perform response upon request from other node.


## Execute nodes

To run rclcpp_pub_sub example, 

```
$ ros2 run tutorial_rclcpp_pub_sub rclcpp_pub_sub
```

To run rclpy example,

```
$ ros2 run tutorial_rclpy_package rclpy_pub_sub 
```

To run rclcpp_service_server example

```
$ ros2 run tutorial_rclcpp_service rclcpp_service_server
```

To send request to server
```
$ ros2 run tutorial_rclpy_package rclpy_service_client
```