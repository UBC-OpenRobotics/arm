#!/usr/bin/env python3.8
from __future__ import print_function
from arm_commander import ArmCommander
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped 
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

############################
# Arm and gripper grasp object test
# the following code is adapated from 
# https://github.com/AIWintermuteAI/ros-moveit-arm/blob/master/my_arm_xacro/pick/pick.py
# with some minor changes
###########################

def degrees_to_radians(joint_angles):
    for i in range(len(joint_angles)):
        joint_angles[i] = joint_angles[i] * np.pi/180.0
    return joint_angles


# Get the gripper posture as a JointTrajectory
def make_gripper_posture(arm_commander : ArmCommander, joint_positions):
    # Initialize the joint trajectory for the gripper joints
    t = JointTrajectory()
    
    # Set the joint names to the gripper joint names
    t.joint_names = arm_commander.robot.get_joint_names(arm_commander.gripper_mvgroup.get_name())
    
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
def make_gripper_translation(arm_commander :ArmCommander, min_dist, desired, vector):
    # Initialize the gripper translation object
    g = GripperTranslation()
    
    # Set the direction vector components to the input
    g.direction.vector.x = vector[0]
    g.direction.vector.y = vector[1]
    g.direction.vector.z = vector[2]
    
    # The vector is relative to the gripper frame
    g.direction.header.frame_id = arm_commander.ARM_REF_FRAME
    
    # Assign the min and desired distances from the input
    g.min_distance = min_dist
    g.desired_distance = desired
    
    return g

# Generate a list of possible grasps
def make_grasps(arm_commander, initial_pose_stamped, allowed_touch_objects):
    gripper_open_joints = degrees_to_radians([-20.0, 0.0, -80.0, 0.0, -80.0])
    gripper_close_joints = degrees_to_radians([-15.0, 0.0, -2.0, 0.0, -2.0])
    arm_pos_joints = degrees_to_radians([-4.0, -29.0, -68.0, -180.0, 83.0])

    # Initialize the grasp object
    g = Grasp()
    
    # Set the pre-grasp and grasp postures appropriately
    g.pre_grasp_posture = make_gripper_posture(arm_commander, gripper_open_joints)
    g.grasp_posture = make_gripper_posture(arm_commander, gripper_close_joints)

    # Set the approach and retreat parameters as desired
    g.pre_grasp_approach = make_gripper_translation(arm_commander, 0.001, 0.001, [0, 0, -1])
    g.post_grasp_retreat = make_gripper_translation(arm_commander, 0.1, 0.15, [0, 0, 1])
    
    # Set the first grasp pose to the input pose
    g.grasp_pose = initial_pose_stamped

    # A list to hold the grasps
    grasps = []
    grasps.append(deepcopy(g))

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
                    print(g)
                    
        print("Generated " + str(len(grasps)) + " poses")
        # Return the list
        return grasps

def grasp_test(arm_commander: ArmCommander,max_attempts=25):
    #clean scene
    arm_commander.scene.remove_world_object("part")
    arm_commander.scene.remove_world_object("table")

    #place scene objects for test
    obj_pos = PoseStamped()
    obj_pos.header.frame_id = arm_commander.ARM_REF_FRAME #todo: test/change reference frame for these objects

    obj_pos.pose.orientation.w = 1.0 #leave other quat. values at default

    obj_pos.pose.position.x = 0.0
    obj_pos.pose.position.y = 0.46
    obj_pos.pose.position.z = 0.025

    table_size = (0.5,0.5,0.05)
    arm_commander.scene.add_box("table", obj_pos, table_size)
    arm_commander.arm_mvgroup.set_support_surface_name("table") #for place operation

    obj_pos.pose.position.x = 0.01
    obj_pos.pose.position.y = 0.29
    obj_pos.pose.position.z = 0.1
    arm_commander.scene.add_cylinder("part", obj_pos, 0.2, 0.04)


    #generate a list of grasp approaches from current eef pose
    #initial pose target
    init_pose = PoseStamped()
    init_pose.header.frame_id = arm_commander.ARM_REF_FRAME
    init_pose.pose = arm_commander.get_end_effector_pose()
    grasps = make_grasps(arm_commander, init_pose, ['part'])

    #attempt pick operation and report success/failure
    result = None
    n_attempts = 0
    while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
        n_attempts += 1
        rospy.loginfo("Pick attempt: " +  str(n_attempts))
        result = arm_commander.arm_mvgroup.pick('part',grasps)
        rospy.sleep(0.2)

def go_ik_test(arm_commander: ArmCommander,iterations):
    """go to random poses generated within reach and test end effector pose accuracy after ik motion complete"""
    poses = []
    for _ in range(iterations):
        arm_commander.go_ik()
        poses.append(arm_commander.pose_goal)
        arm_commander.compare_eef_pose_states()

def go_ik_test_rand(arm_commander: ArmCommander,iterations):
    """go to random poses generated anywhere and test end effector pose accuracy after ik motion complete"""
    poses = []
    pose = Pose()
    angles = np.linspace(-2*np.pi,2*np.pi,iterations)
    positions = np.linspace(-2,2,iterations)

    #generate all poses in range
    for x in positions:
        for y in positions:
            for z in positions:
                for r in angles:
                    for p in angles:
                        for y in angles:
                            quat = quaternion_from_euler(r, p, y)
                            pose.position.x = x
                            pose.position.y = y
                            pose.position.z = z
                            pose.orientation.x = quat[0]
                            pose.orientation.y = quat[1]
                            pose.orientation.z = quat[2]
                            pose.orientation.w = quat[3]
                            poses.append(deepcopy(pose))

    for p in poses:
        print(p)
        arm_commander.go_ik(p)
        arm_commander.compare_eef_pose_states()
        if rospy.is_shutdown():
            break


####################
#Main loop for testing
####################
if __name__ == '__main__':
    gripper_open_joints = degrees_to_radians([-20.0, 0.0, -80.0, 0.0, -80.0])
    gripper_close_joints = degrees_to_radians([-15.0, 0.0, -2.0, 0.0, -2.0])
    arm_pos_joints = degrees_to_radians([-4.0, -29.0, -68.0, -180.0, 83.0])

    arm_commander = ArmCommander()

    arm_commander.go_joint(arm_commander.arm_mvgroup,arm_pos_joints)
    arm_commander.go_joint(arm_commander.gripper_mvgroup,gripper_open_joints)
    grasp_test(arm_commander,10)
