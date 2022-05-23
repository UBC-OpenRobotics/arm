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

# Action Server

Author: Yousif El-Wishahy

Email: yel.wishahy@gmail.com

## ArmCommand.action

The main action for the arm is the ArmCommand action located in /arm_control/action. The action messages have been auto generated with ROS. The action consists of a goal: ```string arm_command ``` and feedback: ```string feedback_message``` as well as a result: ```bool success```. The action goal for arm command is still in the early phase and will likely be changed later on to support more complex goals. 

For now the only supported goal is the string "random pose" which sends a random pose command to the arm, which may result in failure if the pose exceeds physical constraints. This is for testing purposes.

## Running the action server

**NOTE: This runs the "arm_command" node, enviorment should be ran before the "arm_command" node.** 

See [move_config](../../../src/moveit_config/README/README.md) package for specifics of the enviorment.

To run the main node use the following commands.
```bash
source <path to arm>/arm/devel/setup.bash

# runs the enviorment using gazebo when use_gazebo=true. Otherwise false uses Rviz. See specifics in moveit_config pkg
roslaunch moveit_config enviorment.launch use_gazebo:=true 

# running the action server
roslaunch arm_control arm_command.launch
```

## Sending Client Commands to Action Server from Terminal
For testing purposes, we can send an ArmCommandActionGoal from the terminal (acting as client to the server). Of course you muse initialize the server first (check above). In a new terminal : 

```bash
$ source <path to arm>/arm/devel/setup.bash
```

```
$ rostopic pub /arm_command_action_server/goal arm_control/ArmCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  arm_command: 'command string'
  world_pos: [x,y,z]" 
```


Mainly we care about this:
```
goal:
  arm_command: 'command string'
  world_pos: [x,y,z]" 
``` 

## List of Commands

* Random pose: `arm_command: 'random pose' `
* Go to World Position (cartesian coordinate): `arm_command: 'world pos' world_pos: [float32,float32,float32]"  `
