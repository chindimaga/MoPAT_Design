#MoPAT Design Lab - Guining Pertin
#Simulator node(ROS2) - 20-07-20

'''
This node runs pymunk simulation
Subscribed topics:
    /mopat/control/motion_plan_{i}       -   std_msgs/UInt32MultiArrays
    /mopat/control/mrc_output_flags      -   std_msgs/ByteMultiArray
Published topics:
    /mopat/tracking/raw_image            -   sensor_msgs/Image (BGR)
    /mopat/robot/robot_starts            -   std_msgs/UInt32MultiArray
    /mopat/robot/robot_goals             -   std_msgs/UInt32MultiArray
    /mopat/robot/robot_positions         -   std_msgs/UInt32MultiArray
Work:
    Uses pymunk to run simulation and runs robot threads
'''

#Import libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, UInt32, ByteMultiArray
#Others
import sys
import numpy as np
import os
#MoPAT
from mopat_lib import *

class simulator_node(Node):
    def __init__(self):
        #Initialize node
        super().__init__('simulator_node')
        self.get_logger().info("INIT: Started Simulator Node")
        #Subscribers and Publishers
        self.create_subscription("/mopat/control/mrc_output_flags", ByteMultiArray, self.mrc_cb)
        pub_raw = self.create_publisher("/mopat/testbed/raw_image", Image, 1)
        pub_starts = self.create_publisher("/mopat/robot/robot_starts", UInt32MultiArray, queue_size=1)
        pub_goals = self.create_publisher("/mopat/robot/robot_goals", UInt32MultiArray, queue_size=1)
        pub_positions = self.create_publisher("/mopat/robot/robot_positions", UInt32MultiArray, queue_size=1)
        #Local variables
        goals_multiarray = UInt32MultiArray()       #Robot goals - UInt32MultiArray type rosmsg
        positions_multiarray = UInt32MultiArray()   #Robot positions - UInt32MultiArray type rosmsg
        starts_multiarray = UInt32MultiArray()      #Robot starts - UInt32MultiArray type rosmsg
        robot_starts = {}                           #Dict - robot_index : robot start
        robot_goals = {}                            #Dict - robot_index : robot goal
        steps = 50                                  #Simulation steps
        robot_index = 0                             #Number of robots and indexing variable
        got_starts = False                          #Flag - True if user inputs all starts
        got_goals = False                           #Flag - True if user inputs all goals
        started = False                             #Flag - True if simulation started
        got_mouse_click = False                     #Flag - True if mouse clicked
        bridge = CvBridge()                         #Required for rosmsg-cv conversion
        screen_size = (500,500)
        self.self.robot_list = {}                  #Dict - robot_index : ith Robot object
        #Game initialization
        os.environ['SDL_VIDEO_WINDOW_POS'] = "+0,+50"   #Set position
        pygame.init()
        pygame.display.set_caption("MoPAT Multi-Robot Simulator MkV")
        screen = pygame.display.set_mode(screen_size)
        draw_options = pymunk.pygame_util.DrawOptions(screen)
        clock = pygame.time.Clock()
        space = pymunk.Space()                          #Game space
        #Create map
        # generate_empty_map(space)
        # generate_test_map(space)
        generate_random_map(space)
        self.get_logger().info("USER: Enter initial positions now")
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
                    self.get_logger().info("USER: Enter goals now")
                    robot_index = 0                     #Reset index for goals
                    goals_multiarray.data.clear()       #Clear data for goals
                    got_starts = True                   #Flip flag
            #Don't run if all goals not found yet
            if not got_goals:
                #If mouse click found
                if got_mouse_click:
                    #If user inputs starting locations
                    if not got_starts:
                        self.get_logger().info("LOG: Got Robot "+str(robot_index)+
                              " Start: "+str(mouse_x)+"; "+str(mouse_y))
                        #Create robot object with start
                        self.robot_list[robot_index] = Robot(robot_index, space, (mouse_x, mouse_y))
                        starts_multiarray.data.append(mouse_x)
                        starts_multiarray.data.append(mouse_y)
                        #Start subscribers to motion plans for individual robots
                        self.create_subscription("/mopat/control/motion_plan_{0}".format(robot_index),
                                          UInt32MultiArray,
                                          self.robot_list[robot_index].motion_plan_cb)
                        robot_index += 1                #Increment start index
                    #If user inputs goal locations
                    else:
                        #Goal obstacle overlap check
                        if raw_image[screen_size[1]-mouse_y, mouse_x][0] > 128:
                            #Set robot goal
                            robot_goals[robot_index] = (mouse_x, mouse_y)
                            self.robot_list[robot_index].set_goal((mouse_x, mouse_y))
                            goals_multiarray.data.append(mouse_x)
                            goals_multiarray.data.append(mouse_y)
                            self.get_logger().info("LOG: Got Robot "+str(robot_index)+
                                  " Goal: "+str(mouse_x)+"; "+str(mouse_y))
                            robot_index += 1            #Increment goal index
                            if robot_index >= len(self.robot_list):
                                got_goals = True        #Flip flag to start simulation
                                continue
                        else:
                            self.get_logger().info("USER: The point lies within an obstacle")

                    got_mouse_click = False
            #Start simulation if all goals found
            else:
                #Start individual robot threads once
                if not started:
                    self.get_logger().info("LOG: Starting Simulation...")
                    for i in self.robot_list:
                        self.robot_list[i].start()
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
                    self.robot_list[i].draw_goal(screen)
                    pos = self.robot_list[i].get_pos()
                    robot_reached_list.append(self.robot_list[i].robot_reached)
                    #Update robot positions
                    positions_multiarray.data.append(int(pos[0]))
                    positions_multiarray.data.append(int(pos[1]))
                #Stop if all robots reached
                if sum(robot_reached_list) == robot_index and robot_index != 0:
                    self.get_logger().info("LOG: All Robots Reached, Simulation Completed!")
                    break
            #Step up
            pygame.display.flip()
            raw_image = conv2matrix(screen, space, draw_options)
            #Publish raw image
            pub_raw.publish(bridge.cv2_to_imgmsg(raw_image, encoding="passthrough"))
            #Publish robot information
            pub_starts.publish(starts_multiarray)
            pub_goals.publish(goals_multiarray)
            pub_positions.publish(positions_multiarray)
            # pub_num.publish(len(self.robot_list))
            #Clear positions for next step
            positions_multiarray.data.clear()
            clock.tick(steps)
        #End node
        self.get_logger().info("EXIT: Exiting simulation")
        self.get_logger().info("/mopat/user/end_sim", 1)

    def mrc_cb(self, msg):
        '''
        Get MRC control signal
        Arguments:
            data    :   ROS sensor_msgs/ByteMultiArray
        '''
        for i in range(len(msg.data)):
            self.robot_list[i].mrc_flag = msg.data[i]

def main(args=None):
    rclpy.init(args)
    sim_node = simulator_node()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
