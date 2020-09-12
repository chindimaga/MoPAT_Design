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
from threading import Thread
from pygame.locals import *
import pymunk
from pymunk import pygame_util

#Global variables
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

create_node = None

class simulator_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("simulator_node")
        self.get_logger().info("INIT")
        self.pub_raw = self.create_publisher(Image, "/mopat/testbed/raw_image", 2)
        self.pub_starts = self.create_publisher(UInt32MultiArray, "/mopat/robot/robot_starts", 2)
        self.pub_goals = self.create_publisher(UInt32MultiArray, "/mopat/robot/robot_goals", 2)
        self.pub_num = self.create_publisher(UInt32, "/mopat/robot/robot_num", 2)
        #Class variables
        self.goals_multiarray = UInt32MultiArray()      #Robot goals - UInt32MultiArray type rosmsg
        self.positions_multiarray = UInt32MultiArray()  #Robot positions - UInt32MultiArray type rosmsg
        self.starts_multiarray = UInt32MultiArray()     #Robot starts - UInt32MultiArray type rosmsg
        self.robot_num = UInt32()
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
        #Get the simulator running in a way that the node can still receive messages
        #Run the simulator on a different thread while this nodes gets locked in spin
        run_thread = Thread(target=self.run)
        run_thread.daemon = True
        run_thread.start()
        #Get stuck in spin after this

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
                if (event.type == KEYDOWN) and (event.key == K_w) and not self.got_goals:
                    self.get_logger().info("USER: Enter goals now")
                    self.robot_index = 0                     #Reset index for goals
                    self.robot_goals = {}                    #Clear data for goals
                    self.goals_multiarray.data = []
                    self.got_starts = True                   #Flip flag

            #Pre work before getting all goals
            if not self.got_goals:
                #If mouse click found - robot or goal
                if self.got_mouse_click:
                    #Robot
                    if not self.got_starts:
                        # self.robot_list[self.robot_index] = self.add_robot((mouse_x, mouse_y), colors[self.robot_index])
                        self.robot_list[self.robot_index] = Robot(self.robot_index, self.space, (mouse_x, mouse_y))
                        self.get_logger().info("LOG: Got Robot "+str(self.robot_index)+
                              " Start: "+str(mouse_x)+"; "+str(mouse_y))
                        self.starts_multiarray.data.append(mouse_x)
                        self.starts_multiarray.data.append(mouse_y)
                        subs = self.create_subscription(UInt32MultiArray,
                                                "/mopat/control/motion_plan_{0}".format(self.robot_index),
                                                # self.robot_list[self.robot_index].motion_plan_cb, 2)
                                                self.get_cb, 2)
                        subs
                        self.robot_index += 1
                    #Goals
                    else:
                        #Overlap check
                        if self.raw_image[self.screen_size[1]-mouse_y, mouse_x][0] > 128:
                            self.robot_goals[self.robot_index] = (mouse_x, mouse_y)
                            self.robot_list[self.robot_index].set_goal((mouse_x, mouse_y))
                            self.goals_multiarray.data.append(mouse_x)
                            self.goals_multiarray.data.append(mouse_y)
                            self.get_logger().info("LOG: Got Robot "+str(self.robot_index)+
                              " Goal: "+str(mouse_x)+"; "+str(mouse_y))
                            self.robot_index += 1
                            if self.robot_index >= len(self.robot_list):
                                self.got_goals = True
                                continue
                        else:
                            self.get_logger().info("USER: The point lies within an obstacle")
                    self.got_mouse_click = False
            #Start simulation if all goals found
            else:
                if not self.started:
                    self.get_logger().info("LOG: Starting simulation....")
                    for i in self.robot_list:
                        self.robot_list[i].start()
                    self.started = True
            #Update screen
            self.screen.fill((255,255,255))
            self.space.step(1/self.steps)
            self.space.debug_draw(self.draw_options)
            #After each starting location
            if self.got_starts:
                #Draw goal
                for i in range(self.robot_index):
                    self.draw_goal(self.robot_goals[i], i)

            #Step up
            pygame.display.flip()
            #Publish stuff
            #1. Raw image
            self.conv2matrix()
            self.pub_raw.publish(self.bridge.cv2_to_imgmsg(self.raw_image, encoding="passthrough"))
            #2. Start and goal localtion
            self.pub_goals.publish(self.goals_multiarray)
            self.pub_starts.publish(self.starts_multiarray)
            self.robot_num.data = len(self.robot_list)
            self.pub_num.publish(self.robot_num)
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

    def get_cb(self, msg):
        self.get_logger().info("BAS YAHI HONA THA")

    def draw_goal(self, pos, index):
        '''
        Function to draw agent's goal position
        Arguments:
            pos     :   Postion to draw goal
            index   :   Robot index
        '''
        pygame.draw.line(self.screen, pygame.color.THECOLORS[colors[index]],
                         (pos[0]-10, self.screen_size[1]-pos[1]-10),
                         (pos[0]+10, self.screen_size[1]-pos[1]+10),
                         5)
        pygame.draw.line(self.screen, pygame.color.THECOLORS[colors[index]],
                         (pos[0]-10, self.screen_size[1]-pos[1]+10),
                         (pos[0]+10, self.screen_size[1]-pos[1]-10),
                         5)

def add_robot(space, pos, col):
    '''
    Function to generate bots
    Arguments:
        pos     :   Position to spawn vehicle
        col     :   Color of the vehicle
    '''
    #Create robot main body
    body = pymunk.Body(1, pymunk.moment_for_circle(1, 0, 15))
    #Set body properties
    body.position = pos
    body.elasticity = 0
    body.friction = 1
    #Create shape/collision hull
    shape = pymunk.Circle(body, 15)
    shape.color = pygame.color.THECOLORS[col]
    #Show heading side
    heading = pymunk.Circle(body, 5, offset = (10,0))
    heading.color = pygame.color.THECOLORS["white"]
    #Add the object
    space.add(body, shape, heading)
    return shape

class Robot(Thread):
    global create_node
    '''
    The robot class
    Parameters:
        body            : robot body object
        index           : robot's predefined index
        init_pos        : starting location
        goal            : goal location
    '''
    def __init__(self, index, space, pos):
        '''
        Well, initialize
        Arguments:
            index       : robot's predefined index
            space       : pymunk space to work with
            pos         : user defined robot initial position
        '''
        super(Robot, self).__init__()
        self.shape = add_robot(space, pos, colors[index])   #Add robot first
        self.body = self.shape.body                         #Robot body object
        self.index = index                                  #Robot's index
        self.init_pos = pos                                 #Robot's starting position
        self.got_motion_plan = False                        #Flag - True if plan received
        self.robot_reached = False                          #Flag - True if robot reaches dest. or plan fails

    def set_goal(self, goal):
        '''
        Set the goal parameter
        '''
        self.goal = goal

    def get_pos(self):
        '''
        Get robot current position
        '''
        return self.body.position

    def motion_plan_cb(self, data):
        '''
        Get the motion plan for ith robot
        Arguments:
            data    :   ROS std_msgs/UInt32MultiArray
        '''
        #Each time empty the paths
        self.act_pathx = []
        self.act_pathy = []
        #First check if path wasn't found
        for i in range(0,len(data.data)//2):
            #Following the list from normal simulator
            #Adjustments for Pymunk
            self.act_pathx.insert(0, data.data[i*2])
            self.act_pathy.insert(0, simulator_node.screen_size[1] - data.data[i*2+1])
            # self.robot_plan.append((data.data[i*2], data.data[i*2+1]))
        self.got_motion_plan = True     #Flip flag

    def uni_move_robot(self, vel, angle):
        '''
        Function for unicycle model for the robot
        '''
        #Set angle
        self.body.angle = angle
        self.body.velocity = (vel*np.cos(angle),
                         vel*np.sin(angle))

    def basic_robot_controller(self):
        '''
        Basic holonomic robot controller function
        '''
        #Constant velocity, angle controlled
        #Get current position
        (curr_x, curr_y) = self.get_pos()
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #If LSB == 1 : Wait
            #Get angle
            head_angle = np.arctan2(y-curr_y,x-curr_x)
            #Set velocity
            self.uni_move_robot(80, head_angle)
            #Wait until robot reaches the set point
            while not ((int(x-curr_x) == 0) and (int(y-curr_y) == 0)):
                (curr_x, curr_y) = self.get_pos()
        #Stop
        self.holo_move_robot((0,0))
        self.robot_reached = True       #Flip flag

    def run(self):
        '''
        Thread run
        '''
        #Wait until motion plan is received
        while not self.got_motion_plan:
            time.sleep(1)
            continue
        create_node.get_logger().info("ROBOT HERE: Got motion plan")
        #If path wasn't found:
        if self.act_pathx[0] == 9999:
            self.robot_reached = True   #Flip flag
            return 0
        #Otherwise, start controller
        self.basic_robot_controller()

def keep_it_spinning():
    global create_node
    rclpy.spin(create_node)

def main():
    global create_node
    rclpy.init()
    #Create and run
    create_node = simulator_node()
    # run_thread = Thread(target=create_node.run)
    # run_thread.daemon = True
    # run_thread.start()
    spin_thread = Thread(target=keep_it_spinning)
    spin_thread.daemon = True
    try:
        # create_node.get_logger().info("YEH ANDAR AAYA MAIN")
        # rclpy.spin(create_node)
        spin_thread.start()
        create_node.run()
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
