1. Open the file ${ROS PACKAGE NAME}/bin/melodic_project_init_world_1.sh

Locate the path of turtlebot3_simulation package in your system and replace the directory.

sudo cp ../worlds/test_world_1.world ${turtlebot3_simulation path}/turtlebot3_gazebo/worlds/
sudo cp ../worlds/turtlebot3_maze.launch ${turtlebot3_simulation path}/turtlebot3_gazebo/launch/
export TURTLEBOT3_MODEL="burger"
export INIT_X="0.5"
export INIT_Y="0.5"
export INIT_YAW="1.57"
roslaunch turtlebot3_gazebo turtlebot3_maze.launch

2. Launch the gazebo environment 

a. Build the package
b. launch ${ROS PACKAGE NAME}/bin/melodic_project_init_world_1.sh


3. Change the Goal position at ${ROS PACKAGE NAME}/include/PreDefine.cpp (if applicable)

Look for:

#define GOAL_X 4 //target position
#define GOAL_Y 0 //target position
 
Change the number to the grid *INDEX* that you intend to reach.

The index for both X and Y range is [0,9].

4. Launch all nodes 

a. Build the pacakage
b. roslaunch ${ROS PACKAGE NAME} start_all_nodes.launch


