#Guining Pertin
#Motion planner node - 13-05-20

'''
This node generates the motion plan for any robot
given the current position and goal
Subscribed topics:
    mopat/config_space          -   sensor_msgs/Image (Bool)
Published topics:
    mopat/motion_plan           -   std_msgs/String #ToBeChanged
'''

#Import libraries
#ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
#Others
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

#MoPAT alogirithm files
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from algorithms.mopat_astar import Astar

#Global variables
got_config_space = False
index = 0
screen_size = None
start = (50,100)
goal = (450,300)

#CvBridge object required for conversion
bridge = CvBridge()

def config_space_cb(data):
    '''
    Get config_space data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_config_space
    global config_space
    global screen_size
    if not got_config_space:
        config_space = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        config_space = config_space.astype(bool)
        print("LOG: Got configuration space")
        screen_size = config_space.shape
        got_config_space = True

def motion_planner_node():
    '''
    Create notion planner node
    '''
    global config_space
    #Initialize node
    rospy.init_node("motion_planner_node")
    print("LOG: Started A* Motion Plan Generator node")
    #Subscribe to config_space data - Image
    rospy.Subscriber("mopat/config_space", Image, config_space_cb)
    #Publish motion plan
    pub = rospy.Publisher("mopat/motion_plan", String, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Don't generate motion plan until config space is found
        if got_config_space:
            astar_obj = Astar(index, config_space,
                            0, 0, screen_size[1], screen_size[0])
            plan_y, plan_x = astar_obj.find_best_route(screen_size[0]-start[1],
                                                       start[0],
                                                       screen_size[0]-goal[1],
                                                       goal[0])
            motion_plan = (plan_x, plan_y)
            pub.publish("Done")
            rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner_node()
    except rospy.ROSInterruptException:
        pass
