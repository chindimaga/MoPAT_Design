#Guining Pertin
#Multi-robot coordinator node - 16-05-20

'''
This node coordinates the control of all the robots and also starts individual
planner nodes
Subscribed_topics:
    mopat/robot_starts          -   std_msgs/UInt32MultiArray
    mopat/robot_goals           -   std_msgs/UInt32MultiArray
    mopat/robot_positions       -   std_msgs/UInt32MultiArray
    mopat/robot_num             -   std_msgs/UInt32
Starts_nodes:
    motion_planner_nodes for each robot
    robot_controller for each robot
'''
#Import libraries
#ROS
import rospy
from std_msgs.msg import UInt32MultiArray, UInt32
#Other
import numpy as np
#MoPAT

robot_num = 0
robot_starts    = {}
robot_goals     = {}
robot_positions = {}

def robot_positions_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_positions
    for i in range(0,len(data.data),2):
        robot_positions[i] = (data.data[i], data.data[i+1])

def robot_goals_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_goals
    for i in range(0,len(data.data),2):
        robot_goals[i] = (data.data[i], data.data[i+1])

def robot_num_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32
    '''
    global robot_num
    robot_num = data.data

def multi_robot_coordinator_node():
    '''
    Create Multi-Robot Coordinator Node
    '''
    #Initialize node
    rospy.init_node("multi_robot_coordinator_node")
    print("LOG: Starting Multi-Robot Coordinator Node")
    #Set subscribers
    rospy.Subscriber("/mopat/robot_goals", UInt32MultiArray, robot_goals_cb)
    rospy.Subscriber("/mopat/robot_positions", UInt32MultiArray, robot_positions_cb)
    rospy.Subscriber("/mopat/robot_num", UInt32, robot_num_cb)
    while not rospy.is_shutdown():
        #Basically when simulation starts
        if len(robot_goals) == robot_num and robot_num != 0:
            #Start robot_num number of motion planners
            motion_planner_node


if __name__ == "__main__":
    try:
        multi_robot_coordinator_node()
    except rospy.ROSInterruptException:
        pass
