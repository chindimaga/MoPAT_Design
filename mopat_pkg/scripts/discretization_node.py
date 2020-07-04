#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Discretization node - 05-07-20

'''
This node discretizes the configuration space into 1/2^2 scale
for faster path planning
Subscribed topics:
    mopat/tracking/static_config        -   sensor_msgs/Image (Bool)
Published topics:
    mopat/control/discrete_config       -   sensor_msgs/Image (Bool)
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#Others
import sys
import os
import numpy as np

#Global variables
config_space = None         #Predefined global config_space
got_config_space = False    #Flag - True if static_config received
samp_size = 2               #Int  - Sampling size

bridge = CvBridge()         #Required for rosmsg-cv conversion

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
    global samp_size
    #Initialize node
    rospy.init_node("discretization_node")
    rospy.loginfo("INIT: Started Discretization node")
    #Subsribers and Publishers
    rospy.Subscriber("mopat/tracking/static_config", Image, config_space_cb)
    pub = rospy.Publisher("mopat/control/discrete_config", Image, queue_size=1)
    #Get sampling parameter
    samp_size = rospy.get_param("/control/sampling_size")
    #Set rate
    rate = rospy.Rate(1)
    #Discretize!
    while not rospy.is_shutdown():
        #Always check if the simulation is ending
        if rospy.get_param("/user/end_sim"):
            rospy.loginfo("EXIT: Exiting Discretization node")
            sys.exit(0)
        if got_config_space:
            #Subsample 1/2^2
            discrete_config = config_space[::samp_size,::samp_size]
            #Publish stuff
            pub.publish(bridge.cv2_to_imgmsg(discrete_config.astype(np.uint8), encoding="passthrough"))
            got_config_space = False
            #If published, wait for 5s
            rospy.sleep(5)
        rate.sleep()

if __name__ == "__main__":
    try:
        discretization_node()
    except rospy.ROSInterruptException:
        pass
