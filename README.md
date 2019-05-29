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
1. Running on Ubuntu 16.04

2. Installed ROS Kinetic Kame

3. Installed V-REP PRO EDU v.3.5.0

### Setup
Download worksapce to your machine. Sugested download location: /home/

From terminal run commands:
'''bash
cd coverage_wss/coverage_ws/
'''

catkin_make
'''

"build" and "devel" folders should appear when the catkin_make is completed sucesfully

## Running Instructions

### Multi-robot SLAM demo
From terminal run commands:
  
'''bash
cd coverage_wss/coverage_ws/
  
source devel/setup.bash
 
roslaunch turtlebot_launchers master_launch.launch
'''

### Coverage demo
From terminal run commands:
 
'''bash 
cd coverage_wss/coverage_ws/
  
source devel/setup.bash
  
roslaunch turtlebot_launchers coverage.launch
'''
