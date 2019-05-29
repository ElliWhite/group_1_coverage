# group_1_coverage
Repository to hold the workspace for ROCO506 Group Coursework - Multi-robot Coverage using ROS

#### Repository is public due to limited number of collaborators at private level

## About
This repository contains the necessary files to run the following ROCO506 tasks:

Taks 1: Modeling the two turtlebots in V-REP and ROS

Task 2: Multi-Robot 3D SLAM using Octomap including visualization in RVIZ.

Task 3: Implementation in V-REP and ROS of the data structures and interfaces encoding the nodes of the graph. 

Alternative task for full marks: Implementation of the autonomous navigation for one turtlebot, as described in Practical 5th May 2019.

## Instalation Instructions

### Prerequisites
1. Running on Ubuntu 16.04

2. Installed ROS Kinteic Kame

3. Installed V-REP PRO EDU v.3.5.0

### Setup
Download worksapce to your machine. Sugested download location: /home/

Go to: ../coverage_wss/coverage_ws/

Build using "catkin_make" command

## Running Instructions

### Multi-robot SLAM demo
From terminal run command source > ./devel/setup.bash

From same terminal run command > roslaunch turtlebot_launchers master_launch.launch

### Coverage demo
From terminal run command source > ./devel/setup.bash

From same terminal run command > roslaunch turtlebot_launchers coverage.launch
