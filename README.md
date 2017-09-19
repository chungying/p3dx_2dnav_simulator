# p3dx_2dnav_simulator

This repository simulate a p3dx mobile robot in willowgarage.world of Gazebo with Map Server, AMCL, Move Base, RVIZ and a PID controller. The PID controller is for tracking the reference linear velocity provided by local planner of move_base.



## Install

```
$ cd <catkin_ws>/src
$ git clone https://github.com/chungying/p3dx_2dnav_simulator.git
$ cd ..
$ catkin_make
```


### Stuff used to make this:

 * [ua_ros_p3dx](https://github.com/SD-Robot-Vision/PioneerModel.git) for p3dx_gazebo and p3dx_description
 * [ros-planning/navigation](http://wiki.ros.org/navigation/) for p3dx_2dnav
 * [messege_filters](http://wiki.ros.org/message_filters) for p3dx_lin_vel_pid
 * [nav_msgs](http://wiki.ros.org/nav_msgs) for p3dx_lin_vel_pid
 * [geometry_msgs](http://wiki.ros.org/geometry_msgs) for p3dx_lin_vel_pid



