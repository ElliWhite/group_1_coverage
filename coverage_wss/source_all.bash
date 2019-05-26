#! /bin/bash
TURTLEBOT_DRIVERS_WS=~/turtlebot2_wss/turtlebot_ws
TURTLEBOT_SIMULATION_WS=~/turtlebot2_wss/turtlebot_simulation
TURTLEBOT_INTERACTION_WS=~/turtlebot2_wss/turtlebot_interaction
TURTLEBOT_NAVIGATION_WS=~/turtlebot2_wss/turtlebot_navigation_ws
source $TURTLEBOT_DRIVERS_WS/devel/setup.bash --extend
source $TURTLEBOT_SIMULATION_WS/devel/setup.bash --extend
source $TURTLEBOT_INTERACTION_WS/devel/setup.bash --extend
source $TURTLEBOT_NAVIGATION_WS/devel/setup.bash --extend
