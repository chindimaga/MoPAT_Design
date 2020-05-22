#MoPAT Design Lab - Guining Pertin
#Configuration space node - 12-05-20

'''
This node generates the configuration space for the given occ_map
Subscribed topics:
    mopat/occ_map               -   sensor_msgs/Image (Bool)
Published topics:
    mopat/static_config         -   sensor_msgs/Image (Bool)
Work:
    Gets the occupancy map and dilates it to get configuration space
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

#MoPAT algorithm files
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from algorithms import config_space

#Global variables
occ_map = None
got_occ_map = False     #Flag - True if occ_map data received
rad = 20                #Change this with robot size
bridge = CvBridge()     #Required for rosmsg-cv conversion

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
        occ_map = occ_map.astype(bool)
        got_occ_map = True          #Flip flag
        print("LOG: Got occupancy map")

def config_space_node():
    '''
    Create configuration space node
    '''
    global occ_map
    #Initialize node
    rospy.init_node("config_space_node")
    print("LOG: Started Configuration Space Generator node")
    #Subscribers and publishers
    rospy.Subscriber("/mopat/occ_map", Image, occ_map_cb)
    pub = rospy.Publisher("mopat/static_config", Image, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    #Run
    while not rospy.is_shutdown():
        #Don't generate until occ map is found
        if got_occ_map:
            static_config = config_space.gen_config(occ_map, rad)
            pub.publish(bridge.cv2_to_imgmsg(static_config.astype(np.uint8), encoding="passthrough"))
            #Sleep and get occ_map again
            got_occ_map = False     #Flip flag
            rospy.sleep(10)
        rate.sleep()

if __name__ == "__main__":
    try:
        config_space_node()
    except rospy.ROSInterruptException:
        pass
