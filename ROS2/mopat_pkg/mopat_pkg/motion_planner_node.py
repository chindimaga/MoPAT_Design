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
# from mopat_astar_ros import Astar

class motion_planner_node(Node):
    def __init__(self):
        #Intialize
        super().__init__("motion_planner_node")
        self.get_logger().info("INIT")
        self.create_subscription(Image, "/mopat/control/discrete_config", self.discrete_config_cb, 2)
        self.create_subscription(UInt32MultiArray, "/mopat/robot/robot_starts", self.robot_starts_cb, 2)
        self.create_subscription(UInt32MultiArray, "/mopat/robot/robot_goals", self.robot_goals_cb, 2)
        #Class variables
        self.bridge = CvBridge()
        self.robot_starts = {}
        self.robot_goals = {}
        self.samp_size = 2

    def discrete_config_cb(self, msg):
        '''
        Get the discretized space for motion planning
        Arguments:
            msg     :   ROS sensor_msgs/Image
        '''
        self.discrete_config = self.bridge.img_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info("LOG: Got discrete configuration space")
    def robot_starts_cb(self, msg):
        '''
        Get user defined robot starting locations
        Arguments:
            msg   :   ROS std_msgs/UInt32MultiArray
        '''
        for i in range(0, len(msg.data)//2):
            self.robot_starts[i] = (data.data[i*2]//self.samp_size, data.data[i*2+1]//self.samp_size)

    def robot_goals_cb(self, msg):
        '''
        Get user defined robot starting locations
        Arguments:
            msg   :   ROS std_msgs/UInt32MultiArray
        '''
        for i in range(0, len(msg.data)//2):
            self.robot_goals[i] = (data.data[i*2]//self.samp_size, data.data[i*2+1]//self.samp_size)

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = motion_planner_node()
    try:
        rclpy.spin(create_node)
    except:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
