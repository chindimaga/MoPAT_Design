#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Occupancy map node - 15-05-20

'''
This node generates the occupancy map given the raw image
Subscribed topics:
    /mopat/tracking/raw_image         -   sensor_msgs/Image (BGR)
Publised topics:
    /mopat/tracking/occ_map           -   sensor_msgs/Image (Bool)
Work:
    Uses cv2 to threshold raw simulator image and converts to boolean obstacle map
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
#Others
import sys
import cv2
import numpy as np

#Global variables
raw_image = None            #Predifined global raw_image
got_raw_image = False       #Flag - True of raw_image received

bridge = CvBridge()         #Required for rosmsg-cv conversion

def raw_image_cb(data):
    '''
    Get raw_image data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_raw_image
    global raw_image
    if not got_raw_image:
        raw_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        got_raw_image = True                #Flip flag
        rospy.loginfo("LOG: Got raw image")

def occ_map_node():
    '''
    Create occupancy map node
    '''
    #Global variables
    global raw_image
    global got_raw_image
    #Initialize node
    rospy.init_node("occ_map_node")
    rospy.loginfo("INIT: Started Occupancy Map Generator node")
    #Subscribers and publishers
    rospy.Subscriber("/mopat/tracking/raw_image", Image, raw_image_cb)
    pub = rospy.Publisher("/mopat/tracking/occ_map", Image, queue_size=5)
    rate = rospy.Rate(30)
    #Binarize!
    while not rospy.is_shutdown():
        #Always check if the simulation is ending
        if rospy.get_param("/mopat/user/end_sim"): break
        #Don't run until raw_image is found
        if got_raw_image:
            #Get occupancy map
            gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5,5), 0)
            _, occ_map = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
            kernel = np.ones((10,10), np.uint8)
            occ_map = cv2.morphologyEx(occ_map, cv2.MORPH_OPEN, kernel)
            # cv2.imshow("gray", raw_image)
            # cv2.imshow("/occ_map", occ_map)
            occ_map = occ_map.astype(bool)  #Necessary conversion
            #Publish only possible in integer format
            pub.publish(bridge.cv2_to_imgmsg(occ_map.astype(np.uint8), encoding = "passthrough"))
            got_raw_image = False           #Flip flag
            rospy.sleep(5)
            # if cv2.waitKey(1) == ord('q'): break
        rate.sleep()
    #Close the node
    rospy.loginfo("EXIT: Exiting Occupancy Map Generator node")
    sys.exit(0)

if __name__ == "__main__":
    try:
        occ_map_node()
    except rospy.ROSInterruptException:
        pass
