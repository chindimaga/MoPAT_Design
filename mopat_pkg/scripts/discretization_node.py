#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Discretization node - 05-07-20

'''
This node discretizes the configuration space into smaller parts
for faster path planning
Subscribed topics:
    mopat/tracking/static_config        -   sensor_msgs/Image (Bool)
Published topics:
    mopat/control/discrete_space        -   sensor_msgs/Image (Bool)
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#Others
import sys
import os
import numpy

#Global variables
config_space = None
got_config_space = False

bridge = CvBridge()

def config_space_cb(data):
    '''
    Get config_space data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global config_space
    global got_config_space
    if not got_config_space:
        config_space = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        config_space = config_space.astype(bool)
        got_config_space = True
        rospy.loginfo("LOG: Got configuration space")

def discretization_node():
    '''
    Create discretization node
    '''
    #Global variables
    global config_space
    global got_config_space
    #Initialize node
    rospy.init_node("discretization_node")
    rospy.loginfo("INIT: Started Discretization Node")
    #Subsribers and Publishers
    rospy.Subscriber("mopat/tracking/static_config", Image, config_space_cb)
    pub = rospy.Publisher("mopat/tracking/discretization_node", Image, queue_size=1)
    #Set rate
    rate = rospy.Rate(1)
    #Discretize!
    while not rospy.is_shutdown():


if __name__ == "__main__":
    try:
        discretization_node()
    except rospy.ROSInterruptException:
        pass
