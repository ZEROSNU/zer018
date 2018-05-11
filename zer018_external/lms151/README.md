LMS1xx [![Build Status](https://travis-ci.org/clearpathrobotics/LMS1xx.svg?branch=master)](https://travis-ci.org/clearpathrobotics/LMS1xx)
======

# SNU ZERO LIDAR NODE

ROS driver for the SICK LMS1xx family of laser scanners. Originally from [RCPRG](https://github.com/RCPRG-ros-pkg/RCPRG_laser_drivers).

LIDAR IP: 169.254.37.213
Computer ethernet IP: 169.254.37.42

## HOW TO

```
cd catkin_ws/src
git clone https://github.com/dongwan123/lms151
cd ..
catkin_make

```

Run roscore, and rosrun node

```
roscore
```

```
rosrun lms1xx LMS1xx_node _host:=169.254.37.213
rosrun rviz rviz
```

In rviz, set fixed frame as /laser in Global Options

git
