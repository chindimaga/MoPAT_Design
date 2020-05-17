#Guining Pertin
#Motion planner node - 13-05-20

'''
This node generates the motion plan for any robot
given the current position and goal
Subscribed topics:
    mopat/static_config          -   sensor_msgs/Image (Bool)
Published topics:
    mopat/motion_plan           -   std_msgs/String #ToBeChanged
'''

#Import libraries
#ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray, UInt32
#Others
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

#MoPAT alogirithm files
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from algorithms.mopat_astar_ros import Astar

#Global variables
got_static_config = False
all_threads_started = False
static_config = None
screen_size = None
robot_num = 0
robot_starts    = {}
robot_goals     = {}
#CvBridge object required for conversion
bridge = CvBridge()

def config_space_cb(data):
    '''
    Get config_space data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_static_config
    global static_config
    global screen_size
    if not got_static_config:
        static_config = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        static_config = static_config.astype(bool)
        print("LOG: Got static configuration space")
        screen_size = static_config.shape
        got_static_config = True

def robot_starts_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_starts
    for i in range(0,len(data.data)//2):
        robot_starts[i] = (data.data[i*2], data.data[i*2+1])

def robot_goals_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_goals
    for i in range(0,len(data.data)//2):
        robot_goals[i] = (data.data[i*2], data.data[i*2+1])

def robot_num_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32
    '''
    global robot_num
    robot_num = data.data

def motion_planner_node():
    '''
    Create notion planner node
    '''
    global static_config
    global all_threads_started
    #Storage variables
    robot_planners   = {}
    robot_publishers = {}
    #Initialize node
    rospy.init_node("motion_planner_node")
    print("LOG: Started A* Motion Plan Generator node")
    #Subscribe to static_config data - Image
    rospy.Subscriber("mopat/static_config", Image, config_space_cb)
    rospy.Subscriber("/mopat/robot_starts", UInt32MultiArray, robot_starts_cb)
    rospy.Subscriber("/mopat/robot_goals", UInt32MultiArray, robot_goals_cb)
    rospy.Subscriber("/mopat/robot_num", UInt32, robot_num_cb)
    #Publish motion plan
    #Set rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Basically if config_space is found and threads not started yet
        if got_static_config and not all_threads_started:
            #If simulation has started
            if len(robot_goals) == robot_num and robot_num != 0:
                for i in range(robot_num):
                    #New planner for each robot
                    robot_planners[i] = Astar(i, static_config, 0, 0,
                                              screen_size[1], screen_size[0])
                    #Set the start and goal locations
                    robot_planners[i].set_params(robot_starts[i], robot_goals[i], screen_size)
                    #Run threads
                    robot_planners[i].start()
                    #Set publisher
                    robot_publishers[i] = rospy.Publisher("mopat/motion_plan_{0}".format(i),
                                                          String, queue_size=5)
                #All threads started
                all_threads_started = True
        #If done with planners
        if all_threads_started:
            #Check each planner if the path has been generated
            for i in range(robot_num):
                #If path has been generated publish it
                if robot_planners[i].path and robot_planners[i].plan_done:
                    robot_publishers[i].publish("Done")

if __name__ == "__main__":
    try:
        motion_planner_node()
    except rospy.ROSInterruptException:
        pass
