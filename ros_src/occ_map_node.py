#Guining Pertin
#Occupancy map node - 15-05-20

'''
This node generates the occupancy map given the raw image
Subscribed topics:
    mopat/raw_image         -   sensor_msgs/Image (BGR)
Publised topics:
    mopat/occ_map           -   sensor_msgs/Image (Bool)
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

bridge = CvBridge()

raw_image = None
got_raw_image = False

def raw_image_cb(data):
    '''
    Get raw_image data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_raw_image
    global raw_image
    raw_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
    got_raw_image = True

def occ_map_node():
    '''
    Create occupancy map node
    '''
    global got_raw_image
    global raw_image
    rospy.init_node("occ_map_node")
    print("LOG: Started Occupancy Map Generator node")
    #Subscribe to raw image
    rospy.Subscriber("/mopat/raw_image", Image, raw_image_cb)
    #Publish occupancy map
    pub = rospy.Publisher("mopat/occ_map", Image, queue_size=5)
    #Continuously publish data
    while not rospy.is_shutdown():
        #If raw_image not found, don't generate occ_map
        if got_raw_image:
            #Get occupancy map
            _, occ_map = cv2.threshold(cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY),
                                       250, 255, cv2.THRESH_BINARY)
            occ_map = occ_map.astype(bool)
            #Publish data - needs to be converted
            pub.publish(bridge.cv2_to_imgmsg(occ_map.astype(np.uint8), encoding = "passthrough"))
            got_raw_image = False

if __name__ == "__main__":
    try:
        occ_map_node()
    except rospy.ROSInterruptException:
        pass
