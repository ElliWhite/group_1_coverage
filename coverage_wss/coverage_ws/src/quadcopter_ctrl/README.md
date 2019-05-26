SETUP
-----

- install VREP and then link the main exe "vrep.sh" in your /usr/bin path with the following commands
$ cd usr/bin
$ ln -sf <your path to VREP dir>/vrep.sh . 
- install ROS indigo
- enter in the clone dir and execute 
$ cd src
$ catkin_init_workspace 
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ cd devel/lib/quadcopter_ctrl/
$ ln -sf ../../../src/quadcopter_ctrl/Input .

================================================================================
LAUNCH A SIMULATION TEST
------------------------

In order to launch a simulation test
1) open a terminal and execute
$ roscore
2) in V-REP open the scene 3Rob_freeEnvironm.ttt (it's in "src/quadcopter_ctrl" dir).
2) open a new terminal and execute
$ roslaunch quadcopter_ctrl swarmNodeCount_3.launch input:=map.txt minVis:=1

================================================================================
ALGORITHMS
------------------------

- quadcopterRosCtrl.cpp offline version 
- quadNodeCount.cpp online version

================================================================================

More fun below...

================================================================================
================================================================================


VRep-Ros Quadcopter Swarm
=========================


Summary
-------

This is a framework to control a series of quad-copters simulated in V-REP by using ROS publisher/subscribers, to completely cover a given terrain using different coverage algorithms. By using the roslaunch files you can configure how many robots you want to use for the coverage.
The terrain to be covered is given as input to the executables which start with the "quad" prefix and is represented as a binary occupancy grid (e.g. zero for accessible point, 1 for not accessible point).
All the algorithms are publishing the positions to be followed on ROS topics that are read by the V-REP simulator using the *.ttt scenes provided in the package.

**Videos**:

https://www.youtube.com/watch?v=cW3FLxkx9K4

https://www.youtube.com/watch?v=5RocmrFCXQg

Compiling and running
=====================

To build the following package you will need [ROS](http://wiki.ros.org/ROS) to be installed and clone this repository in the src of a new [catkin package](http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage).
To use the scenarios (the *.ttt files) you have to install (preferrably the latest version of) the V-REP simulator, available from the [Coppelia Robotics](http://www.coppeliarobotics.com/) website.

Compile V-REP Ros Library
-------------------------
Once you've downloaded it follow this steps to compile the V-REP ROS Plugin:

**1.** Copy the **vrep_common** and **vrep_plugin** folder from *vrep-folder/programming/ros_packages* to *catkin_ws/src*

**2.** Depending on the ROS version you are using, change CMakeLists.txt of vrep_plugin to point to the correct libraries adding:
```
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
```

**3.** Compile the plugin with `catkin_make`

**4.** Copy the builded plugin from */devel/lib/libv_repExtRos.so* to the V-REP directory

**5.** Done! :smile:

Compile the catkin package
--------------------------
**1.** Be sure you have cloned the repo in the "catkin_ws/src" folder. **Rename** the cloned folder (VRepRosQuadSwarm) to **quadcopter_ctrl** (since it's the actual name of the catkin package).

**2.** Compile once with `catkin_make` (to be invoked in catkin_ws/).

How to run
----------

**1.** Create the necessary folders to create the following path:

*catkin_ws/devel/lib/quadcopter_ctrl/<b>Input/Grids/</b>*
     
Inside this folder create a file (for example "map_file.txt") containing a binary occupancy matrix, for example:
```
    0 0 0 0 
    0 0 1 1
    0 0 1 1
    0 0 0 0
```
That represents a map with a square obstacle on the right.

**2.** Now launch `roscore` in a terminal, and V-REP in another terminal.

**3.** In V-REP open the scene *3Rob_freeEnvironm.ttt* (it's in the root of the repository).

**4.** In a third terminal use one of the roslaunch files to run a coverage, using as input the file that you created before, for example:
```
$ roslaunch quadcopter_ctrl swarmNodeCount_3.launch input:=map_file.txt
```

**5.** Now the quadcopter should start performing the coverage in V-REP.

About the launch files
----------------------
When using your own map file, adding quadcopter to the scene, or any other customization, pay attention to the parameters present in the roslaunch file. Inside the launch file there is a description of these input parameters. For the online algorithms for example the parameters are the following:

```
argv[1]: Quadcopter # to control
argv[2]: Input file
argv[3]: zHeight of flight
argv[4]: Control Mode ('sim' or 'asctec')
argv[5]: STARTNODE, index of first node
argv[6]: Min #visits for each node
```

So when creating your own map keep an eye on STARTNODE for example, because if it is set to a node with an obstacle the code will not work (is like starting from inside the obstacle). The same applies if you set STARTNODE to a number that is greater than the number of nodes *n* in the map (in the map of the example *n*=16, so the starting node has to be less than 16).

Details
=======

The are five algorithms developed, that can be divided in two areas: offline and online algorithms.


Offline Algorithms
------------------

Paths generated offline, before quad-copters start moving:

1. VRP Greedy using A*

2. VRP Greedy using Floyd-Warshall


In quadcopterRosCtrl.cpp, you can choose which of the two algorithm to use by editing the declared class in the main. Quadcopters follow some paths generated by the Greedy Algorithm run in kernelNode, which solves the Vehicle routing problem using the VRPGreedy class to cover all the environment simulated in V-Rep.


Online Algorithms
-----------------

Paths generated online while robots are moving:

1. Node Counting

2. Learning Real-Time A*

3. PatrolGRAPH*


In quadNodeCount.cpp the path is generated using the NodeCount class to find the nearest node with the smallest number of visits. Visit count maps are updated in a callback function using a topic common to all the quadcopter-controller nodes, named "/updateNodeCount". In this way a quadcopter sends a message that will be received by all the quadcopters including itself. This kind of behaviour was designed to make the code easily scalable. The quadLRTA.cpp has a similar behaviour.


