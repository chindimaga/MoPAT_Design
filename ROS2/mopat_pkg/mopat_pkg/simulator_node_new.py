#MoPAT Design Lab - Guining Pertin
#Simulator node(ROS2) - 08-07-20

'''
This node runs pymunk simulation
Subscribed topics:

Published topics:

Work:
    Uses pymunk to run simulation and runs robot threads
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, UInt32, ByteMultiArray
#Others
import numpy as np
import os
import time
import pygame
from threading import Thread
from pygame.locals import *
import pymunk
from pymunk import pygame_util
#MoPAT libraries
from algorithms.mopat_lib import *

#Global variables
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

robot_list = {}
sim_node = None

class simulator_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("simulator_node")
        self.get_logger().info("INIT")
        self.pub_raw = self.create_publisher(Image, "/mopat/testbed/raw_image", 2)
        self.pub_starts = self.create_publisher(UInt32MultiArray, "/mopat/robot/robot_starts", 2)
        self.pub_goals = self.create_publisher(UInt32MultiArray, "/mopat/robot/robot_goals", 2)
        self.pub_num = self.create_publisher(UInt32, "/mopat/robot/robot_num", 2)
        self.create_subscription(UInt32MultiArray, "/mopat/control/motion_plan_0", self.motion_plan_cb, 2)

    # def subscribe_plan(self, topic_name, callback):
    #     self.create_subscription(UInt32MultiArray, topic_name, self.motion_plan_cb, 2)

    def motion_plan_cb(self, data):
        '''
        Get the motion plan for ith robot
        Arguments:
            data    :   ROS std_msgs/UInt32MultiArray
        '''
        self.get_logger().info("Got callback")
        # #Each time empty the paths
        # self.act_pathx = []
        # self.act_pathy = []
        # #First check if path wasn't found
        # for i in range(0,len(data.data)//2):
        #     #Following the list from normal simulator
        #     #Adjustments for Pymunk
        #     self.act_pathx.insert(0, data.data[i*2])
        #     self.act_pathy.insert(0, simulator_node.screen_size[1] - data.data[i*2+1])
        #     # self.robot_plan.append((data.data[i*2], data.data[i*2+1]))
        # self.got_motion_plan = True     #Flip flag

def run_simulation():
    '''
    Function to run the simulation
    '''
    global robot_list
    global sim_node
    #Local variables
    goals_multiarray = UInt32MultiArray()       #Robot goals - UInt32MultiArray type rosmsg
    positions_multiarray = UInt32MultiArray()   #Robot positions - UInt32MultiArray type rosmsg
    starts_multiarray = UInt32MultiArray()      #Robot starts - UInt32MultiArray type rosmsg
    robot_num = UInt32()                        #Robot numbers
    robot_starts = {}                           #Dict - robot_index : robot start
    robot_goals = {}                            #Dict - robot_index : robot goal
    steps = 50                                  #Simulation steps
    robot_index = 0                             #Number of robots and indexing variable
    got_starts = False                          #Flag - True if user inputs all starts
    got_goals = False                           #Flag - True if user inputs all goals
    started = False                             #Flag - True if simulation started
    got_mouse_click = False                     #Flag - True if mouse clicked
    screen_size = (500,500)
    bridge = CvBridge()                         #Required for rosmsg-cv conversion
    #Pygame setup
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    space = pymunk.Space()                          #Game space
    #Create map
    # generate_empty_map(space)
    # generate_test_map(space)
    generate_random_map(space)
    sim_node.get_logger().info("USER: Enter initial positions now")
    #Simulate!
    while 1:
        for event in pygame.event.get():
            #Exiting
            if event.type == QUIT:
                break
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                break
            #Get user input
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                mouse_y = screen_size[1] - mouse_y  #Pygame pymunk conversion
                got_mouse_click = True              #Flip flag
            if (event.type == KEYDOWN) and (event.key == K_w) and not got_goals:
                sim_node.get_logger().info("USER: Enter goals now")
                robot_index = 0                     #Reset index for goals
                goals_multiarray.data = []       #Clear data for goals
                got_starts = True                   #Flip flag
        #Don't run if all goals not found yet
        if not got_goals:
            #If mouse click found
            if got_mouse_click:
                #If user inputs starting locations
                if not got_starts:
                    sim_node.get_logger().info("LOG: Got Robot "+str(robot_index)+
                          " Start: "+str(mouse_x)+"; "+str(mouse_y))
                    #Create robot object with start
                    robot_list[robot_index] = Robot(robot_index, space, (mouse_x, mouse_y))
                    starts_multiarray.data.append(mouse_x)
                    starts_multiarray.data.append(mouse_y)
                    #Start subscribers to motion plans for individual robots
                    # sim_node.create_subscription(UInt32MultiArray,"/mopat/control/motion_plan_{0}".format(robot_index),robot_list[robot_index].motion_plan_cb, 2)
                    # sim_node.subscribe_plan("/mopat/control/motion_plan_{0}".format(robot_index),robot_list[robot_index].motion_plan_cb)
                    robot_index += 1                #Increment start index
                #If user inputs goal locations
                else:
                    #Goal obstacle overlap check
                    if raw_image[screen_size[1]-mouse_y, mouse_x][0] > 128:
                        #Set robot goal
                        robot_goals[robot_index] = (mouse_x, mouse_y)
                        robot_list[robot_index].set_goal((mouse_x, mouse_y))
                        goals_multiarray.data.append(mouse_x)
                        goals_multiarray.data.append(mouse_y)
                        sim_node.get_logger().info("LOG: Got Robot "+str(robot_index)+
                              " Goal: "+str(mouse_x)+"; "+str(mouse_y))
                        robot_index += 1            #Increment goal index
                        if robot_index >= len(robot_list):
                            got_goals = True        #Flip flag to start simulation
                            continue
                    else:
                        sim_node.get_logger().info("USER: The point lies within an obstacle")

                got_mouse_click = False
        #Start simulation if all goals found
        else:
            #Start individual robot threads once
            if not started:
                sim_node.get_logger().info("LOG: Starting Simulation...")
                for i in robot_list:
                    robot_list[i].start()
                started = True
        #Update screen
        screen.fill((255,255,255))
        space.step(1/steps)
        space.debug_draw(draw_options)
        #After getting starting locations
        if got_starts:
            #Draw goal and update robot_reached_list
            robot_reached_list = []
            for i in range(robot_index):
                robot_list[i].draw_goal(screen)
                pos = robot_list[i].get_pos()
                robot_reached_list.append(robot_list[i].robot_reached)
                #Update robot positions
                positions_multiarray.data.append(int(pos[0]))
                positions_multiarray.data.append(int(pos[1]))
            #Stop if all robots reached
            if sum(robot_reached_list) == robot_index and robot_index != 0:
                sim_node.get_logger().info("LOG: All Robots Reached, Simulation Completed!")
                break
        #Step up
        pygame.display.flip()
        raw_image = conv2matrix(screen, space, draw_options)
        #Publish raw image
        sim_node.pub_raw.publish(bridge.cv2_to_imgmsg(raw_image, encoding="passthrough"))
        #Publish robot information
        sim_node.pub_starts.publish(starts_multiarray)
        sim_node.pub_goals.publish(goals_multiarray)
        robot_num.data = len(robot_list)
        sim_node.pub_num.publish(robot_num)
        clock.tick(steps)
    #End node
    sim_node.get_logger().info("EXIT: Exiting simulation")

def main():
    global sim_node
    rclpy.init()
    #Game initialization
    os.environ['SDL_VIDEO_WINDOW_POS'] = "+0,+50"
    pygame.init()
    pygame.display.set_caption("MoPAT Multi-Robot Simulator MkV")
    #Node
    sim_node = simulator_node()
    #Thread for running
    run_thread = Thread(target=run_simulation)
    run_thread.daemon = True
    try:
        run_thread.start()
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        sim_node.get_logger().info("EXIT")
        sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
