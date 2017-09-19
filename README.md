# p3dx_2dnav_simulator

This repository simulate a p3dx mobile robot in willowgarage.world of Gazebo with Map Server, AMCL, Move Base, RVIZ and a PID controller. The PID controller is for tracking the reference linear velocity provided by local planner of move_base.


## Install

```
$ cd <catkin_ws>/src
$ git clone https://github.com/chungying/p3dx_2dnav_simulator.git
$ cd ..
$ catkin_make
```

## Execution

1.Running the original settings for a p3dx robot without PID contorller
  - p3dx in Willog Garage world at (0, 0, 0) pose.
```
$ roslaunch p3dx_2dnav p3dx_2dnav.launch
```
  - p3dx at (-4, -4, 3.14159) pose.
```
$ roslaunch p3dx_2dnav p3dx_2dnav.launch ip_x:=-4 ip_y:=-4 ip_a:=3.14159
```

2.Running the simulation with PID controller. Default pose is (0, 0, 0). Default PID gains is (0.8, 0.001, 0.01)
```
$ roslaunch p3dx_lin_vel_pid p3dx_2dnav_pid.launch ip_x:=-4 ip_y:=-4 ip_a:=3.14159 Kp:=1 Ki:=0 Kd:=0
```


### Stuff used to make this:

 * [ua_ros_p3dx](https://github.com/SD-Robot-Vision/PioneerModel.git) for p3dx_gazebo and p3dx_description
 * [ros-planning/navigation](http://wiki.ros.org/navigation/) for p3dx_2dnav
 * [messege_filters](http://wiki.ros.org/message_filters), [nav_msgs](http://wiki.ros.org/nav_msgs), and [geometry_msgs](http://wiki.ros.org/geometry_msgs) for p3dx_lin_vel_pid
 * [gmapping](http://wiki.ros.org/gmapping) and [map_server](http://wiki.ros.org/map_server) for building willowgarage

