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

# NOTE: Node adapted from https://moveit.picknik.ai/galactic/doc/examples/move_group_python_interface/move_group_python_interface_tutorial.html

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

print("current pose: ", move_group.get_current_pose().pose)

for i in range(30):
    print(f"[{i}] resetting position")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0 #pi / 4
    joint_goal[2] = 0
    joint_goal[3] = 0 #pi / 4

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    print(f"[{i}] go to pose position")



    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.000000
    pose_goal.position.y = -0.11826354658766348
    pose_goal.position.z = 0.35000000000000000000000000

    pose_goal.orientation.x = 0.7074552029005028
    pose_goal.orientation.y = -2.705443833806829e-07
    pose_goal.orientation.z = 0.0005450587139244744
    pose_goal.orientation.w = 0.7067579775283286



    # move_group.set_goal_tolerance(0.1)
    move_group.set_planning_time(60)
    move_group.set_num_planning_attempts(5)
    move_group.set_pose_target(pose_goal)


    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()


# # check if pose is valid source: https://answers.ros.org/question/240723/fail-aborted-no-motion-plan-found-no-execution-attempted/
# plan = group.plan()
# if not plan.joint_trajectory.points:
#     # Error
