#!/usr/bin/env python3.8
from __future__ import print_function
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

# docs: https://github.com/ros-planning/moveit/tree/ee48dc5cedc981d0869352aa3db0b41469c2735c
# Author: Yousif El-Wishahy

def all_close(goal, actual, tolerance):
    """
    From: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

##############################
# arm command class to be called by robot loop
##############################
class ArmCommander(object):
    plan = None # current joint trajectory plan
    pose_goal = None #current pose goal

    sample_time_out   = 10            # time out in seconds for Inverse Kinematic search
    sample_attempts   = 10             # num of planning attempts 
    goal_tolerance    = 0.1 

    ARM_REF_FRAME = "arm1_base_fixed"
    GRIPPER_REF_FRAME = "arm1_palm"
    WORLD_FRAME = "world"
 
    def __init__(self):
        '''
        @brief init arm command object, moveit commander, scene and movegroups for arm and arm gripper
        '''
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("arm_commander", anonymous=True)

        self.robot:RobotCommander = moveit_commander.RobotCommander()
        self.scene:PlanningSceneInterface = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)

        self.arm_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander("arm")
        self.arm_mvgroup.set_planning_time(self.sample_time_out)
        self.arm_mvgroup.set_num_planning_attempts(self.sample_attempts)
        self.arm_mvgroup.set_goal_tolerance(self.goal_tolerance)
        self.arm_mvgroup.set_max_velocity_scaling_factor(1)
        self.arm_mvgroup.set_max_acceleration_scaling_factor(1)

        self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander("gripper")
        
        #debug print
        print("============ Reference frame: %s" % self.arm_mvgroup.get_planning_frame())

        print("============ Pose Reference frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        eef_link = self.arm_mvgroup.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        print("============ Robot Groups:", self.robot.get_group_names())

        print("============ Robot Links:", self.robot.get_link_names())

        print("============ Robot Joints:", self.robot.get_active_joint_names())

        print("============ Printing robot state")
        print(self.robot.get_current_state())

        print("============ Printing MoveIt interface description")
        print(self.arm_mvgroup.get_interface_description())

        self.register_gripper_joint_states()

    def register_gripper_joint_states(self):
        joint_vals_min = [0]
        joint_vals_max = [0]
        for joint_name in self.gripper_mvgroup.get_joints()[1:]:
            joint = self.robot.get_joint(joint_name)
            joint_vals_min.append(joint.bounds()[0])
            joint_vals_max.append(joint.bounds()[1])

        #todo: test these remembered joint states
        self.gripper_mvgroup.remember_joint_values("open", values=joint_vals_max)
        self.gripper_mvgroup.remember_joint_values("close", values=joint_vals_min)
        
    def go_ik(self, pose_goal=None):
        '''
        @brief Makes the head of the arm go to a position and orientation
        specified by goal, blocks until either inverse kinematics fails 
        to find path or it has reached the goal within tolerace.

        @param pose_goal : geometry_msgs\Pose contains desired 
        position and orientation for gripper

        if pose_goal param is not passed, random pose goal will be generated

        @return bool success flag
        '''

        if pose_goal == None:
            self.pose_goal = self.arm_mvgroup.get_random_pose().pose
        else:
            self.pose_goal = pose_goal
        self.plan = self.arm_mvgroup.plan(self.pose_goal)

        if (self.plan[0]):
            self.arm_mvgroup.go(joints=None, wait=True)
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()
        return self.plan[0]

    @staticmethod
    def go_joint(group : MoveGroupCommander, goal=None):
        """
        @brief Makes the group's joints go to joint angles specified by goal,
        blocks until goal is reached within tolerance or exit if goal joint
        angles is impossible.

        @param group (MoveGroupCommander) : Move group object , either arm or gripper

        @param goal (JointState): JointState object that specifies joint angle goals
        returns: void
        """
        if goal == None:
            goal = group.get_random_joint_values()
        group.go(joints=goal, wait=True)
        group.stop()
    
    def get_end_effector_pose(self):
        return self.robot.get_link(self.arm_mvgroup.get_end_effector_link()).pose().pose

    def print_robot_cartesian_state(self):
        print('ROBOT CARTESIAN STATE\n')
        for link_name in self.robot.get_link_names():
            link = self.robot.get_link(link_name)
            print(link.name(),'\n',link.pose().pose)
    
    def compare_eef_pose_states(self):
        """
        @brief print goal pose and actual pose and if it meets self.goal_tolerance
        """
        if self.plan[0]:
            goal = self.pose_goal
            actual = self.get_end_effector_pose()

            print('\n\n')
            print("End effector goal:\n",goal)
            print('\n')
            print("End effector result:\n",actual)
            print('\n\n')
            print('Goal within tolerance: %s : %s' % (self.goal_tolerance,all_close(goal,actual,self.goal_tolerance)))