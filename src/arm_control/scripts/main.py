#!/usr/bin/env python3.8
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
import math

# docs: https://github.com/ros-planning/moveit/tree/ee48dc5cedc981d0869352aa3db0b41469c2735c
##############################
# Init
##############################

group_name = "arm"      
sample_time_out   = 60               # time out in seconds for Inverse Kinematic search
sample_attempts   = 5                # num of planning attempts 
goal_tolerance    = 0.001

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("main", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planning_time(sample_time_out)
move_group.set_num_planning_attempts(5)
move_group.set_goal_tolerance(goal_tolerance)
move_group.set_max_velocity_scaling_factor(1)
move_group.set_max_acceleration_scaling_factor(0.3)



##############################
# functions
##############################


def go_ik(goal):
    """
    aka: "go inverse kinematics"
    Makes the head of the arm go to a position and orientation
    specified by goal, blocks until either inverse kinematics fails 
    to find path or it has reached the goal within tolerace.

    Args: 
        goal (Pose):  pose object that specifies 
            arm head's goal transform
    returns: 
        bool: success
    """

    plan = move_group.plan(goal)
    if (plan[0]):
        move_group.go(joints=None, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    return plan[0]

def go_joint(goal):
    """
    Makes the arm's joints go to joint angles specified by goal,
    blocks until goal is reached within tolerance or exit if goal joint
    angles is impossible.

    Args:
        goal (JointState): JointState object that specifies joint angle goals
    returns: void
    """

    move_group.go(joints=goal, wait=True)
    move_group.stop()

def f(goal, tolerance):
    current_pose = move_group.get_current_pose()


##############################
# main loop
##############################

while (True):
    print(f"Success: {go_ik(move_group.get_random_pose().pose)}")
    

# for i in range(1):
#     pose_goal = geometry_msgs.msg.Pose()
#     pose_goal.position.x = -0.3
#     pose_goal.position.y = 0.23
#     pose_goal.position.z = 0.45

#     print(f"success: {go_ik(pose_goal)} xyz: ({pose_goal.position.x}, {pose_goal.position.y}, {pose_goal.position.z})")


