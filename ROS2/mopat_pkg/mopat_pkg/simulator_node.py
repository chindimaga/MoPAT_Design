#MoPAT Design Lab - Guining Pertin
#Simulator node(ROS2) - 22-07-20

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
from pygame.locals import *
import pymunk
from pymunk import pygame_util

class simulator_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("simulator_node")
        self.get_logger().info("INIT")
        self.pub_raw = self.create_publisher(Image, "/mopat/testbed/raw_image", 2)

        #Class variables
        self.goals_multiarray = UInt32MultiArray()      #Robot goals - UInt32MultiArray type rosmsg
        self.positions_multiarray = UInt32MultiArray()  #Robot positions - UInt32MultiArray type rosmsg
        self.starts_multiarray = UInt32MultiArray()     #Robot starts - UInt32MultiArray type rosmsg
        self.robot_starts = {}                          #Dict - robot_index : robot start
        self.robot_goals = {}                           #Dict - robot_index : robot goal
        self.steps = 50                                 #Simulation steps
        self.robot_index = 0                            #Number of robots and indexing variable
        self.got_starts = False                         #Flag - True if user inputs all starts
        self.got_goals = False                          #Flag - True if user inputs all goals
        self.started = False                            #Flag - True if simulation started
        self.got_mouse_click = False                    #Flag - True if mouse clicked
        self.bridge = CvBridge()                        #Required for rosmsg-cv conversion
        self.robot_list = {}                            #Dict - robot_index : ith Robot object
        self.screen_size = (500,500)                    #Default screen_size - (int, int)
        self.end_sim = False                            #Flag - Default ros param to end sim
        #Game initialization
        os.environ['SDL_VIDEO_WINDOW_POS'] = "+0,+50"
        pygame.init()
        pygame.display.set_caption("MoPAT Multi-Robot Simulator MkV")
        self.screen = pygame.display.set_mode(self.screen_size)
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.clock = pygame.time.Clock()
        self.space = pymunk.Space()
        #Create map
        self.generate_random_map()

    def run(self):
        '''
        Function to start the simulation
        '''
        #Run simulation
        while 1:
            #First check if sim needs to run
            if self.end_sim:
                break
            #Check events
            for event in pygame.event.get():
                #Exiting
                if event.type == QUIT:
                    self.end_sim = True
                elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                    self.end_sim = True
                #Get user input
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    mouse_y = self.screen_size[1] - mouse_y #Pygame pymunk conversion
                    self.got_mouse_click = True             #Flip flag
                if (event.type == KEYDOWN) and (event.key == K_w) and not got_goals:
                    rospy.loginfo("USER: Enter goals now")
                    self.robot_index = 0                    #Reset index for goals
                    goals_multiarray.data.clear()           #Clear data for goals
                    self.got_starts = True                  #Flip flag
            #Update screen
            self.screen.fill((255,255,255))
            self.space.step(1/self.steps)
            self.space.debug_draw(self.draw_options)

            #Step up
            pygame.display.flip()
            #Publish stuff
            #1. Raw image
            self.conv2matrix()
            self.pub_raw.publish(self.bridge.cv2_to_imgmsg(self.raw_image, encoding="passthrough"))

            #End
            self.clock.tick(self.steps)
        self.get_logger().info("EXIT")

    def conv2matrix(self):
        '''
        Function to convert pygame window to numpy matrix
        '''
        window_matrix = np.array(pygame.surfarray.array3d(self.screen))
        window_matrix = window_matrix[..., ::-1]
        self.raw_image = np.flip(np.rot90(window_matrix, 1),0)

    def add_static_obstacle(self, pos, size):
        '''
        Function to generate static obstacle
        '''
        #Create static body
        body = pymunk.Body(body_type = pymunk.Body.STATIC)
        body.position = pos[0] + size[0]/2, pos[1] + size[1]/2
        if body.position[0] < 0 or body.position[1] < 0: return 0
        #Create box shape
        shape = pymunk.Poly.create_box(body, size, 0.0)
        shape.color = pygame.color.THECOLORS["black"]
        #Add the object
        self.space.add(body, shape)

    def generate_random_map(self):
        '''
        Function to generate a random map for simulation
        '''
        #Create borders
        self.add_static_obstacle((0,0), (5,500))
        self.add_static_obstacle((0,0), (500,5))
        self.add_static_obstacle((495,0), (5,500))
        self.add_static_obstacle((0,495), (500,5))
        #Consider map as a 25x25 matrix using const size obstacles
        obstacle_size = (20,20)
        for y in range(25):
            prob_no = (7+np.random.choice(3, p = [0.1,0.1,0.8]))/10
            map_x = np.random.choice(2, size = 25, p = [prob_no, 1-prob_no])
            for x in range(25):
                if map_x[x]:
                    self.add_static_obstacle((25*x,25*y), obstacle_size)

def main():
    rclpy.init()
    #Create and run
    create_node = simulator_node()
    try:
        create_node.run()
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
    #Close node on exit
    create_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
