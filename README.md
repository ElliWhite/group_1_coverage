# group_1_coverage
Repository to hold the workspace for ROCO506 Group Coursework - Multi-robot Coverage using ROS

#### Repository is public due to limited number of collaborators at private level

## About
This repository contains the necessary files to run the following ROCO506 tasks:

Taks 1: Modeling the two turtlebots in V-REP and ROS

Task 2: Multi-Robot 3D SLAM using Octomap including visualization in RVIZ.

Task 3: Implementation in V-REP and ROS of the data structures and interfaces encoding the nodes of the graph. 

Alternative task for full marks: Implementation of the autonomous navigation for one turtlebot, as described in Practical 5th May 2019.

## Installation Instructions

### Prerequisites
1. Ubuntu 16.04 LTS

2. ROS Kinetic

3. V-REP PRO EDU v.3.5.0

### Setup
Clone repository to your machine. 

From terminal run commands:
```bash
cd ~/coverage_wss/coverage_ws/

catkin_make
```

## Running Instructions

### Multi-Robot Autonomous Navigation
From terminal run commands:

```bash
roscore
```
In a new terminal:

```bash
cd <YOUR_PATH_TO_VREP>/
./vrep.sh
```
Open the scene turtlebot_maze.ttt from *group_1_coverage/coverage_wss/coverage_ws/src/turtlebot_simulation/src/vrep_turtlebot_simulation/scene/*

Press play.

In a new terminal:
  
```bash
cd ~/coverage_wss/coverage_ws/
  
source devel/setup.bash

roslaunch turtlebot_launchers master_launch.launch
```
Move the interactive markers to a desired location. Right-click and select "Select Goal".

### Multi-Robot Coverage
From terminal run commands:
 
```bash 
cd ~/coverage_wss/coverage_ws/
  
source devel/setup.bash
  
roslaunch turtlebot_launchers coverage.launch
```
Press play in V-REP.

## Images
Transform frames in RViz:
![TF](https://i.imgur.com/Kc4c70h.png)

Rectified and coloured point cloud from Kinect RGBD sensor:
![colouredPCL](https://i.imgur.com/qSa876P.png)

Using Octomap, normal estimation and traversibility analysis:
![octomap&normal&trav](https://i.imgur.com/Sljl6AM.png)

Multi-Robot 3D Mapping Pt1:
![mr3dmapping](https://i.imgur.com/T01LrZ0.png)

Multi-Robot 3D Mapping Pt2:
![mr3dmapping2](https://i.imgur.com/VSuZAVz.png)

Multi-Robot 3D Mapping Pt3:
![mr3dmapping3](https://i.imgur.com/eoSKhhc.png)

Multi-Robot 3D Mapping Pt4:
![mr3dmapping4](https://i.imgur.com/kmEROo4.png)

Multi-Robot 3D Mapping Pt5:
![mr3dmapping5](https://i.imgur.com/hWN8Hu1.png)

Maze in V-REP:
![maze](https://i.imgur.com/1Z4s08f.png)


## Videos

[Multi-Robot Autonomous Navigation](https://www.youtube.com/watch?v=ZRH_pJarqpo) (Low FPS is due to heavy load on laptop)

[Multi-Robot Coverage](https://www.youtube.com/watch?v=m6F9Eu0oMMM)

[3D Mapping Using Octomap](https://www.youtube.com/watch?v=rV_tpvMzQqk)
