#MoPAT Design Lab
#Configuration space node - 12-05-20

'''
This node generates the configuration space for the given occ_map
Subscribed topics:
    mopat/occ_map           -   sensor_msgs/Image (Bool)
    mopat/user_input        -   std_msgs/String #ToBeChanged
Published topics:
    mopat/config_space      -   sensor_msgs/Image (Bool)
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
#Others
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

#MoPAT alogirithm files
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from algorithms import config_space

#Global variables
occ_map = None
got_occ_map = False
rad = 20
#CvBridge object required for conversion
bridge = CvBridge()

def occ_map_cb(data):
    '''
    Get occ_map data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_occ_map
    global occ_map
    if not got_occ_map:
        occ_map = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        #Convert to boolean
        occ_map = occ_map.astype(bool)
        print("LOG: Got occupancy map")
        got_occ_map = True

def user_input_cb(data):
    '''
    Decode user_input data
    Arguments:
        data    :   ROS std_msgs/String
    '''
    return 0

def config_space_node():
    '''
    Create configuration space node
    '''
    global occ_map
    #Initialize node
    rospy.init_node("config_space_node")
    print("LOG: Started Configuration Space Generator node")
    #Subscribe to occupancy map data - Image
    rospy.Subscriber("/mopat/occ_map", Image, occ_map_cb)
    #Subscribe to user input - String
    rospy.Subscriber("/mopat/user_input", String, user_input_cb)
    #Publish configuration space
    pub = rospy.Publisher("mopat/config_space", Image, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Don't generate until occ map is found
        if got_occ_map:
            #Generate static configuration space
            static_config = config_space.gen_config(occ_map, rad)
            #Publish data - needs to be converted
            pub.publish(bridge.cv2_to_imgmsg(static_config.astype(np.uint8), encoding="passthrough"))
        rate.sleep()

if __name__ == "__main__":
    try:
        config_space_node()
    except rospy.ROSInterruptException:
        pass
