# Arm Control Package

Author: Jonathan Qiao

Email: jonathanqfxw@gmail.com

# Descripton:

**This package is meant to be an envorment to develope the manipulation/control aspect of the arm.**

The arm is controlled with python script located in ```arm/src/arm_control/scripts``` directory. The control starts in ```main.py``` which starts ```main``` ROS node.

# Usage:

## Running the control

**NOTE: This runs the main node, enviorment should be ran before the main node.** 

See [move_config](../../../src/moveit_config/README/README.md) package for specifics of the enviorment.

To run the main node use the following commands.
```bash
source <path to arm>/arm/devel/setup.bash

# runs the enviorment using gazebo when use_gazebo=true. Otherwise false uses Rviz. See specifics in moveit_config pkg
roslaunch moveit_config enviorment.launch use_gazebo:=true 

# running the main node
roslaunch arm_control main.launch
```