#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import JointState

def listenerCallback(data):
    # data.name 
    # data.position
    # data.velocity
    # data.effort
    index = 1
    rospy.loginfo('%s : %s ', data.name[index], data.position[index])

def listener():
    rospy.init_node('joint_reader', anonymous=False)

    
    rospy.Subscriber('joint_states', JointState, listenerCallback)

    rospy.spin()


if __name__ == '__main__':
    listener()