#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Motion planner node - 13-05-20

'''
This node generates the motion plan for robots given starts and goals
Subscribed topics:
    mopat/tracking/static_config          -   sensor_msgs/Image (Bool)
    mopat/robot/robot_starts              -   std_msgs/UInt32MultiArray
    mopat/robot/robot_goals               -   std_msgs/UInt32MultiArray
Published topics:
    mopat/control/motion_plan_{i}         -   std_msgs/UInt32MultiArrays
    mopat/control/motion_plans_done       -   std_msgs/Bool
Work:
    Gets config_space, starts and goals and uses A* to get motion plans
'''

#Import libraries
#ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray, UInt32, Bool
#Others
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

#MoPAT alogirithm files
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from algorithms.mopat_astar_ros import Astar

#Global variables
motion_plans_done = Bool()          #Flag - Bool type rosmsg
static_config = None                #Predefined global static_config
screen_size = None                  #Predefined global static_config
robot_starts = {}                   #Dict - robot_index : robot_start
robot_goals  = {}                   #Dict - robot_index : robot_goal
motion_plans = {}                   #Dict - robot_index : robot_plan
got_static_config = False           #Flag - True if static_config received
all_planners_started = False        #Flag - True if all motion_planners started
motion_plans_done.data = False      #Flag - True if all motion plans done

bridge = CvBridge()                 #Required for rosmsg-cv conversion

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
        screen_size = static_config.shape       #Set screen size using config_space
        got_static_config = True                #Flip flag
        rospy.loginfo("LOG: Got static configuration space")

def robot_starts_cb(data):
    '''
    Get user defined robot starting locations
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_starts
    for i in range(0,len(data.data)//2):
        robot_starts[i] = (data.data[i*2], data.data[i*2+1])

def robot_goals_cb(data):
    '''
    Get user defined robot goal locations
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_goals
    for i in range(0,len(data.data)//2):
        robot_goals[i] = (data.data[i*2], data.data[i*2+1])

def convplan2multiarray(robot_index, px, py):
    '''
    Function to convert motion_plan list to rosmsg type
    Arguments:
        robot_index :   index of robot to save plans to
        px          :   list to convert x coordinates from
        py          :   list to convert y coordinates from
    '''
    global motion_plans
    #Set as UInt32MultiArray
    motion_plans[robot_index] = UInt32MultiArray()
    #Append data in x,y form
    for i in range(len(px)):
        motion_plans[robot_index].data.append(px[i])
        motion_plans[robot_index].data.append(py[i])

def motion_planner_node():
    '''
    Create motion planner node
    '''
    #Global variables
    global static_config
    global all_planners_started
    global motion_plans
    global motion_plans_done
    #Local variables
    robot_planners   = {}       #Dict - robot_index : robot A* object
    robot_publishers = {}       #Dict - robot_index : robot publisher object
    #Initialize node
    rospy.init_node("motion_planner_node")
    rospy.loginfo("INIT: Started A* Motion Plan Generator node")
    #Subscribers and publishers
    rospy.Subscriber("mopat/tracking/static_config", Image, config_space_cb)
    rospy.Subscriber("mopat/robot/robot_starts", UInt32MultiArray, robot_starts_cb)
    rospy.Subscriber("mopat/robot/robot_goals", UInt32MultiArray, robot_goals_cb)
    pub_done = rospy.Publisher("mopat/control/motion_plans_done", Bool, queue_size = 1)
    #Set rate
    rate = rospy.Rate(1)
    #Plan!
    while not rospy.is_shutdown():
        #Always check the number of robots
        robot_num = rospy.get_param("/user/robot_num")
        #Don't start until all static config is found and planners aren't started
        if got_static_config and not all_planners_started:
            #Don't run if simulation hasn't started yet
            if len(robot_goals) == robot_num and robot_num != 0:
                for i in range(robot_num):
                    #New planner for each robot
                    robot_planners[i] = Astar(i, static_config, 0, 0,
                                              screen_size[1], screen_size[0])
                    robot_planners[i].set_params(robot_starts[i], robot_goals[i], screen_size)
                    #Start planners and publishers
                    robot_planners[i].start()
                    robot_publishers[i] = rospy.Publisher("mopat/control/motion_plan_{0}".format(i),
                                                          UInt32MultiArray, queue_size=5)
                all_planners_started = True     #Flip flag - after planners are started
        #Don't run until all planners are set
        if all_planners_started:
            motion_plans_completed = 0          #Published plans tracker
            for i in range(robot_num):
                #If ith robot's planning done and path generated - publish
                if robot_planners[i].path and robot_planners[i].plan_done:
                    convplan2multiarray(i, robot_planners[i].py, robot_planners[i].px)
                    robot_publishers[i].publish(motion_plans[i])
                    motion_plans_completed += 1 #Increment
                #Else if ith robot's planning done but path not generated - publish not_done
                elif not robot_planners[i].path and robot_planners[i].plan_done:
                    convplan2multiarray(i, [99999], [99999])
                    robot_publishers[i].publish(motion_plans[i])
                    motion_plans_completed += 1 #Increment
            #If all plans are generated
            if motion_plans_completed == robot_num:
                motion_plans_done.data = True   #Flip flag
            pub_done.publish(motion_plans_done) #Necesary evil
            #If threads started, publish at a slower rate
            rospy.sleep(5)
            continue
        pub_done.publish(motion_plans_done)     #Necessary evil
        rate.sleep()

if __name__ == "__main__":
    try:
        motion_planner_node()
    except rospy.ROSInterruptException:
        pass
