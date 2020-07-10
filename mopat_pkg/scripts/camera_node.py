#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Camera output node - 10-07-20

'''
This node gets the raw image from the camera and publishes it
Published topics:
    /mopat/tracking/raw_image           -   sensor_msgs/Image (int8)
Work:
    Uses OpenCV to turn on the camera and get raw image
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#Others
import sys
import os
import cv2
import numpy as np

bridge = CvBridge()     #Required for rosmsg-cv conversion

def camera_node():
    '''
    Create camera output node
    '''
    #Initialize node
    rospy.init_node("camera_node")  #user /mopat/tracking ns
    rospy.loginfo("INIT: Started Camera node")
    #Publisher
    pub = rospy.Publisher("/mopat/tracking/raw_image", Image, queue_size=2)
    rate = rospy.Rate(60)
    #Start camera
    cap = cv2.VideoCapture(0)
    #Lights! Camera! Action!
    while not rospy.is_shutdown():
        #Always check if the simulation is ending
        # if rospy.get_param("/mopat/user/end_sim"): break
        #Get camera frame
        _, frame = cap.read()
        pub.publish(bridge.cv2_to_imgmsg(frame.astype(np.uint8), encoding="passthrough"))
        #Show frame - should be commented often
        cv2.imshow("/raw_image", frame)
        if cv2.waitKey(1) == ord('q'): break
        rate.sleep()
    #Close the node
    rospy.loginfo("EXIT: Exiting Camera node")
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == "__main__":
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
