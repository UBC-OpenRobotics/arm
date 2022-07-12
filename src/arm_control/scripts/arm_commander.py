#!/usr/bin/env python3.8
from __future__ import print_function
from mimetypes import init
from time import sleep
from six.moves import input
import sys
import rospy
import moveit_commander
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
import math
import actionlib
from math import pi, tau, dist, fabs, cos
import geometry_msgs
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

    sample_time_out   = 60            # time out in seconds for Inverse Kinematic search
    sample_attempts   = 5             # num of planning attempts 
    goal_tolerance    = 0.01
 
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
        self.arm_mvgroup.set_max_acceleration_scaling_factor(0.3)

        self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander("gripper")
        
        #debug print
        print("============ Reference frame: %s" % self.arm_mvgroup.get_planning_frame())

        print("============ Pose Reference frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        eef_link = self.arm_mvgroup.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        print("============ Robot Groups:", self.robot.get_group_names())

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
    def go_joint(group, goal=None):
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


####################
# Testing functions
# todo: move to another file later
####################
def go_ik_test(arm_commander: ArmCommander,iterations):
    poses = []
    for _ in range(iterations):
        arm_commander.go_ik()
        poses.append(arm_commander.pose_goal)
        arm_commander.compare_eef_pose_states()
    for pose in poses:
        arm_commander.go_ik(pose)
        arm_commander.compare_eef_pose_states()

def pick_object_test(arm_commander: ArmCommander):
    # clean the scene
    arm_commander.scene.remove_world_object("pole")
    arm_commander.scene.remove_world_object("table")
    arm_commander.scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position.x = 0.7
    p.pose.position.y = -0.4
    p.pose.position.z = 0.85
    p.pose.orientation.w = 1.0
    arm_commander.scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.175
    arm_commander.scene.add_box("table", p, (0.5, 1.5, 0.35))

    p.pose.position.x = 0.6
    p.pose.position.y = -0.7
    p.pose.position.z = 0.5
    arm_commander.scene.add_box("part", p, (0.15, 0.1, 0.3))


############################
# Arm and gripper grasp object test
# the following code is adapated from 
# https://github.com/AIWintermuteAI/ros-moveit-arm/blob/master/my_arm_xacro/pick/pick.py
# with some minor changes
###########################
def grasp_test(arm_commander: ArmCommander,max_attempts=25):
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(arm_commander, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = arm_commander.robot.get_joint_names(arm_commander.arm_mvgroup.get_name())
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = [1.0]
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(arm_commander, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = arm_commander.arm_mvgroup.get_pose_reference_frame()
        print('gripper frame: %s\n' % g.direction.header.frame_id)
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(arm_commander, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = make_gripper_posture(arm_commander, [0,0,0,0,0,0])
        g.grasp_posture = make_gripper_posture(arm_commander, [2,2,2,2,2,2])
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = make_gripper_translation(arm_commander, 0.1, 0.1, [0, 1, 0])
        g.post_grasp_retreat = make_gripper_translation(arm_commander, 0.1, 0.15, [1, 0, 0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        ideal_roll = 0
        ideal_pitch = 0
        ideal_yaw = 0
        
        step_size = 0.1
        idx = 0.1
        idx_roll = ideal_roll + idx
        idx_pitch = ideal_pitch + idx
        idx_yaw = ideal_yaw + idx
        roll_vals = []
        pitch_vals = []
        yaw_vals = []
        while idx >= -0.1:
            roll_vals.append(idx_roll)
            pitch_vals.append(idx_pitch)
            yaw_vals.append(idx_yaw)
            idx -= step_size
            idx_roll -= step_size
            idx_pitch -= step_size
            idx_yaw -= step_size

        # A list to hold the grasps
        grasps = []
        
        # Generate a grasp for each roll pitch and yaw angle
        for r in roll_vals:
            for y in yaw_vals:
                for p in pitch_vals:
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(r, p, y)
                    
                    # Set the grasp pose orientation accordingly
                    g.grasp_pose.pose.orientation.x = q[0]
                    g.grasp_pose.pose.orientation.y = q[1]
                    g.grasp_pose.pose.orientation.z = q[2]
                    g.grasp_pose.pose.orientation.w = q[3]
                    
                    # Set and id for this grasp (simply needs to be unique)
                    g.id = str(len(grasps))
                    
                    # Set the allowed touch objects to the input list
                    g.allowed_touch_objects = allowed_touch_objects
                    
                    # Don't restrict contact force
                    g.max_contact_force = 0
                    
                    # Degrade grasp quality for increasing pitch angles
                    g.grasp_quality = 1.0 - abs(p)
                    
                    # Append the grasp to the list
                    grasps.append(deepcopy(g))
                    
        print("Generated " + g.id + " poses")
        # Return the list
        return grasps
    
    #clean scene
    arm_commander.scene.remove_world_object("part")
    arm_commander.scene.remove_world_object("table")

    #place part
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame() #todo: test/change reference frame for these objects

    print('placing part in scene with reference frame %s\n' % p.header.frame_id)
    p.pose.orientation.w = 1.0 #leave other quat. values at default
    p.pose.position = arm_commander.get_end_effector_pose().position

    p.pose.position.x -= 0.3
    p.pose.position.z = 0.1
    arm_commander.scene.add_box("table", p, (0.5, 0.5, 0.25))

    p.pose.position.z = 0.25
    arm_commander.scene.add_box("part", p, (0.15, 0.1, 0.15))

    print('part placed\n')

    #generate a list of grasp approaches from current eef pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = arm_commander.arm_mvgroup.get_pose_reference_frame()
    init_pose.pose = arm_commander.get_end_effector_pose()

    grasps = make_grasps(arm_commander, init_pose, 'part')

    #attempt pick operation and report success/failure
    result = None
    n_attempts = 0
    while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
        n_attempts += 1
        rospy.loginfo("Pick attempt: " +  str(n_attempts))
        result = arm_commander.arm_mvgroup.pick('part',grasps)
        rospy.sleep(0.2)


####################
#Main loop for testing
####################
if __name__ == '__main__':
    arm_commander = ArmCommander()

    grasp_test(arm_commander,10)
