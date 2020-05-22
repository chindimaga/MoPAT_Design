#MoPAT Design Lab - Guining Pertin
#Occupancy map node - 15-05-20

'''
This node generates the occupancy map given the raw image
Subscribed topics:
    mopat/raw_image         -   sensor_msgs/Image (BGR)
Publised topics:
    mopat/occ_map           -   sensor_msgs/Image (Bool)
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
        print("LOG: Got raw image")

def occ_map_node():
    '''
    Create occupancy map node
    '''
    #Global variables
    global raw_image
    global got_raw_image
    #Initialize node
    rospy.init_node("occ_map_node")
    print("LOG: Started Occupancy Map Generator node")
    #Subscribers and publishers
    rospy.Subscriber("/mopat/raw_image", Image, raw_image_cb)
    pub = rospy.Publisher("mopat/occ_map", Image, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    #Binarize!
    while not rospy.is_shutdown():
        #Don't run until raw_image is found
        if got_raw_image:
            #Get occupancy map
            _, occ_map = cv2.threshold(cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY),
                                       250, 255, cv2.THRESH_BINARY)
            occ_map = occ_map.astype(bool)  #Necessary conversion
            pub.publish(bridge.cv2_to_imgmsg(occ_map.astype(np.uint8), encoding = "passthrough"))
            #Sleep and get raw_image again
            got_raw_image = False           #Flip flag
            rospy.sleep(5)
        rate.sleep()

if __name__ == "__main__":
    try:
        occ_map_node()
    except rospy.ROSInterruptException:
        pass
