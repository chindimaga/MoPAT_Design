#MoPAT Design Lab
#Configuration space node - 14-05-20

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
#Other
import sys
import os
import numpy as np

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
        #Get boolean map
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
    rospy.init_node("config_space_node", anonymous=True)
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
            #Generate configuration space
            config_space = config_space.gen_config(occ_map, rad).astype(np.uint8)
            #Publish data
            pub.publish(bridge.cv2_to_imgmsg(config_space, encoding="passthrough"))
        rate.sleep()

if __name__ == "__main__":
    try:
        config_space_node()
    except rospy.ROSInterruptException:
        pass
