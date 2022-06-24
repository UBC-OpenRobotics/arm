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
import actionlib
from arm_control.msg import ArmCommandAction,ArmCommandFeedback, ArmCommandResult

# docs: https://github.com/ros-planning/moveit/tree/ee48dc5cedc981d0869352aa3db0b41469c2735c

##############################
# Global parameters
##############################
ros_node_name = "arm_command"  

sample_time_out   = 60               # time out in seconds for Inverse Kinematic search
sample_attempts   = 20                # num of planning attempts 
goal_tolerance    = 0.001

group_name = "arm"    
gripper_group_name = "gripper"


##############################
# ik / moveit functions
##############################
def go_ik(move_group,goal):
    """
    aka: "go inverse kinematics"
    Makes the head of the arm go to a position and orientation
    specified by goal, blocks until either inverse kinematics fails 
    to find path or it has reached the goal within tolerace.

    Args: communicate
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

def go_joint(group, goal):
    """
    Makes the arm's joints go to joint angles specified by goal,
    blocks until goal is reached within tolerance or exit if goal joint
    angles is impossible.

    Args:
        goal (JointState): JointState object that specifies joint angle goals
    returns: void
    """

    group.go(joints=goal, wait=True)
    group.stop()

def plan_cartesian(move_group,dest=[1,1,1]):
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z = dest[2]  
    wpose.position.y = dest[1]  
    wpose.position.x = dest[0]
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan, fraction
##############################
# action server class
##############################

class ArmCommandActionClass(object):
    # Create messages that are used to publish feedback/result
    _feedback = ArmCommandFeedback()
    _result = ArmCommandResult()
    
    move_group = None
    gripper_group = None
    robot = None
    scene = None

    def __init__(self, name):
        # This will be the name of the action server which clients can use to connect to.
        self._action_name = name
        self.init_moveit()

        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self._as = actionlib.SimpleActionServer(self._action_name, ArmCommandAction, execute_cb=self.execute_cb, auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")
    
    def init_moveit(self):
        '''
        @brief init the moveit commander, scene and movegroups for arm and arm gripper
        '''
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planning_time(sample_time_out)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_goal_tolerance(goal_tolerance)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
    
    def print_robot_cartesian_state(self):
        print('ROBOT CARTESIAN STATE\n')
        for link_name in self.robot.get_link_names()[1:4]:
            link = self.robot.get_link(link_name)
            print(link.name(),'\n',link.pose().pose)

    def execute_cb(self, goal):
        '''
        @brief execute callback function for action server, that takes action goal of
            type ArmCommandAction and processes the message/values into a moveit pose to execute using go_ik()
        '''
        rospy.loginfo('[arm command server] in arm action server execute command')
        feedback_message = ""
        flag = False
        print(self.move_group.get_current_state())
        self.print_robot_cartesian_state()

        if(goal.arm_command == 'random pose'):
            rospy.loginfo('[arm command server] executing random pose command...')
            pose_goal = self.move_group.get_random_pose().pose
            print(pose_goal)
            feedback_message = f"Success: {go_ik(self.move_group, pose_goal)}"
            flag = True
        elif("world pos" in goal.arm_command):
            rospy.loginfo('[arm command server] executing world position command...')
            pose_goal = goal.pose

            print(pose_goal)

            feedback_message = f"Success: {go_ik(self.move_group, pose_goal)}"
            flag = True

        if not flag:
            feedback_message = "Invalid arm command!"

        self._feedback.feedback_message = feedback_message
        self._as.publish_feedback(self._feedback)


        if('ABORTED' in feedback_message):
            self._result.success = False
        else:
            self._result.success = True
            self._as.set_succeeded(self._result)

####################
#Main
####################
if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node(ros_node_name)

    # Create an instance of the action server here.
    server = ArmCommandActionClass(rospy.get_name())
    rospy.spin()