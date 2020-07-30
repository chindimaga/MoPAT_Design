#MoPAT Design Lab - Guining Pertin
#Motion planner node(ROS2) - 31-07-20

'''
This node generates the motion plan for robots given starts and goals
Subscribed topics:
    /mopat/tracking/discrete_config        -   sensor_msgs/Image (Bool)
    /mopat/robot/robot_starts              -   std_msgs/UInt32MultiArray
    /mopat/robot/robot_goals               -   std_msgs/UInt32MultiArray
Published topics:
    /mopat/control/motion_plan_{i}         -   std_msgs/UInt32MultiArrays
    /mopat/control/motion_plans_done       -   std_msgs/Bool
Work:
    Gets config_space, starts and goals and uses A* to get motion plans
'''


#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, UInt32, Bool
#Others
import numpy as np
import time
from mopat_astar_ros import Astar

class motion_planner_node(Node):
    def __init__(self):
        #Intialize
        super.__init__("motion_planner_node")
        self.get_logger().info("INIT")
        
