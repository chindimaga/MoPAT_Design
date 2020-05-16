#Guining Pertin
#Multi-robot coordinator node - 16-05-20

'''
This node coordinates the control of all the robots and also starts individual
planner nodes
Subscribed_topics:
    mopat/robot_starts          -   std_msgs/UInt32MultiArray
    mopat/robot_goals           -   std_msgs/UInt32MultiArray
    mopat/robot_positions       -   std_msgs/UInt32MultiArray
Starts_nodes:
    motion_planner_nodes for each robot
    robot_controller for each robot
'''
#Import libraries
#ROS
import rospy
from std_msgs.msg import UInt32MultiArray
#Other
import numpy as np

robot_starts    = {}
robot_goals     = {}
robot_positions = {}

def robot_starts_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_starts
    for i in range(0,len(data.data),2):
        robot_starts[i] = (data.data[i], data.data[i+1])

def robot_goals_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_goals
    for i in range(0,len(data.data),2):
        robot_goals[i] = (data.data[i], data.data[i+1])

def robot_positions_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_positions
    for i in range(0,len(data.data),2):
        robot_positions[i] = (data.data[i], data.data[i+1])

def multi_robot_coordinator_node():
    '''
    Create Multi-Robot Coordinator Node
    '''
    #Initialize node
    rospy.init_node("multi_robot_coordinator_node")
    print("LOG: Starting Multi-Robot Coordinator Node")
    #Set subscribers
    rospy.Subscriber("/mopat/robot_starts", UInt32MultiArray, robot_starts_cb)
    rospy.Subscriber("/mopat/robot_goals", UInt32MultiArray, robot_goals_cb)
    rospy.Subscriber("/mopat/robot_positions", UInt32MultiArray, robot_positions_cb)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        multi_robot_coordinator_node()
    except rospy.ROSInterruptException:
        pass
