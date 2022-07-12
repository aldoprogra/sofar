# Software-Architecture-Assignment

## Description

https://user-images.githubusercontent.com/80604899/178480042-e4f722f4-85d7-425a-aff9-821eceec8a39.mp4

## Installation

1- Install Ubnutu 18.04 with ROS and TIAGO, by following this documentation [installation](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) and go to your workspace.
2- clone the repository
3- put and replace tiago_trajectory_controller package inside  /src/tiago_tutorials/
4- put and replace run_motion_python_node.py node inside /src/play_motion/play_motion/scripts/

Do this commands: 

```
source ./devel/setup.bash
```

```
roscore 
```

```
catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)
```
```
rosparam load dump.yaml
```

5 - open, source and run this commands in three terminals :

```
 roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=empty
```
```
rosrun tiago_tutorials/tiago_trajectory_controller webcam.py
```
```
rosrun play_motion run_motion_python_node.py
```

## Credits
Prof. Simone Macciò
Students: SINATRA GESUALDO  || ABDELGHANI BAKOUR || ZHOUYANG HONG     
