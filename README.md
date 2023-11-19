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

## tutorial_rclcpp_package

**tutorial_rclcpp_package** constains executable **rclcpp_example**.
Also includes
- msg/ Custom message
- srv/ 


## Execute nodes

To run rclcpp example, 

```
$ ros2 run tutorial_rclcpp_package rclcpp_example
```

To run rclpy example,

```
$ ros2 run tutorial_rclpy_package rclpy_example 
```