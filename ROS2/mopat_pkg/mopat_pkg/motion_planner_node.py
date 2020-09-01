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
from threading import Thread
from mopat_pkg.mopat_astar_ros import Astar

class motion_planner_node(Node):
    def __init__(self):
        #Intialize
        super().__init__("motion_planner_node")
        self.get_logger().info("INIT")
        self.create_subscription(Image, "/mopat/control/discrete_config", self.discrete_config_cb, 2)
        self.create_subscription(UInt32MultiArray, "/mopat/robot/robot_starts", self.robot_starts_cb, 2)
        self.create_subscription(UInt32MultiArray, "/mopat/robot/robot_goals", self.robot_goals_cb, 2)
        self.create_subscription(UInt32, "/mopat/robot/robot_num", self.robot_num_cb, 2)
        self.pub_done = self.create_publisher(Bool, "mopat/control/motion_plans_done", 2)
        #Class variables
        self.bridge = CvBridge()
        self.robot_starts = {}
        self.robot_goals = {}
        self.samp_size = 2
        self.robot_num = 0
        self.motion_plans_completed = 0
        self.all_planners_started = False
        self.motion_plans_done = Bool()
        self.motion_plans_done.data = False
        self.motion_plans = {}
        self.robot_planners = {}
        self.robot_publishers = {}
        self.screen_size = None

    def run(self):
        '''
        Get the motion plans and publish them
        '''
        self.get_logger().info("Running planner!")
        while 1:
            #Run if planners not started yet
            if not self.all_planners_started:
                #Don't run until simulation starts
                if len(self.robot_goals) == self.robot_num and self.robot_num != 0:
                    for i in range(self.robot_num):
                        #New planner for each robot
                        self.robot_planners[i] = Astar(i, self.discrete_config, 0, 0,
                                                  self.screen_size[1], self.screen_size[0])
                        self.robot_planners[i].set_params(self.robot_starts[i], self.robot_goals[i],
                                                            self.screen_size)
                        self.robot_planners[i].start()
                        self.robot_publishers[i] = self.create_publisher(UInt32MultiArray,
                                                    "/mopat/control/motion_plan_{0}".format(i),2)
                    self.all_planners_started = True
                #If simulation not started yet, break out for now
                else: break
            #Run when all planners are set
            else:
                self.motion_plans_completed = 0
                for i in range(self.robot_num):
                    if self.robot_planners[i].plan_done:
                        self.convplan2multiarray(i)
                        self.robot_publishers[i].publish(self.motion_plans[i])
                        self.motion_plans_completed += 1
                        #In case the plan wasn't found, it will return [99999], [99999]
                #If all plans generated, break out
                if self.motion_plans_completed == self.robot_num:
                    self.get_logger().info("Plans completed")
                    self.motion_plans_done.data = True
                    self.pub_done.publish(self.motion_plans_done)
                    break
            self.pub_done.publish(self.motion_plans_done)


    def discrete_config_cb(self, msg):
        '''
        Get the discretized space for motion planning
        Arguments:
            msg     :   ROS sensor_msgs/Image
        '''
        self.get_logger().info("LOG: Got discrete configuration space")
        self.discrete_config = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.screen_size = self.discrete_config.shape
        self.run()

    def robot_starts_cb(self, msg):
        '''
        Get user defined robot starting locations
        Arguments:
            msg   :   ROS std_msgs/UInt32MultiArray
        '''
        for i in range(0, len(msg.data)//2):
            self.robot_starts[i] = (msg.data[i*2]//self.samp_size, msg.data[i*2+1]//self.samp_size)

    def robot_goals_cb(self, msg):
        '''
        Get user defined robot goal locations
        Arguments:
            msg   :   ROS std_msgs/UInt32MultiArray
        '''
        for i in range(0, len(msg.data)//2):
            self.robot_goals[i] = (msg.data[i*2]//self.samp_size, msg.data[i*2+1]//self.samp_size)

    def robot_num_cb(self, msg):
        '''
        Get number of robots required for debugging purposes
        Arguments:
            msg     :   ROS std_msgs/UInt32
        '''
        self.robot_num = msg.data

    def convplan2multiarray(self, robot_index):
        '''
        Function to convert motion_plan list to rosmsg type
        Arguments:
            robot_index :   index of robot to save plans to
        '''
        self.motion_plans[robot_index] = UInt32MultiArray()
        px = self.robot_planners[robot_index].px
        py = self.robot_planners[robot_index].py
        #Append data in x,y form
        for i in range(len(px)):
            self.motion_plans[robot_index].data.append(px[i]*self.samp_size)
            self.motion_plans[robot_index].data.append(py[i]*self.samp_size)

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = motion_planner_node()
    try:
        rclpy.spin(create_node)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
