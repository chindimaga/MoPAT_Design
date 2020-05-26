#MoPAT Design Lab - Guining Pertin
#MoPAT library functions and classes - Prev sim

'''
This library contains often used functions and classes based on the prev
"Non+ROS" simulator - more often pymunk functions
'''

#Import libraries
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import numpy as np
import time
from threading import Thread

#Global variables
screen_size = (500,500)             #Default screen_size
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

def add_robot(space, pos, col):
    '''
    Function to generate bots
    Arguments:
        space   : pymunk space object
        pos     : (posx, posy) tuple
        col     : string color name
    Returns:
        body    : robot_body object
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
    heading = pymunk.Circle(body, 4, offset = (10,0))
    heading.color = pygame.color.THECOLORS["black"]
    #Add the object
    space.add(body, shape, heading)
    return shape

def add_static_obstacle(space, pos, size):
    '''
    Function to generate static obstacle
    Arguments:
        space   : pymunk space object
        pos     : (posx, posy) tuple
        size    : (sizex, sizey) tuple
    Returns:
        shape   : static obstacle shape
    '''
    #Create static body
    body = pymunk.Body(body_type = pymunk.Body.STATIC)
    body.position = pos[0] + size[0]/2, pos[1] + size[1]/2
    if body.position[0] < 0 or body.position[1] < 0: return 0
    #Create box shape
    shape = pymunk.Poly.create_box(body, size, 0.0)
    shape.color = pygame.color.THECOLORS["white"]
    #Add the object
    space.add(body, shape)

def generate_random_map(space):
    '''
    Function to generate random maps for simulations
    Arguments:
        space   : pymunk space object
    '''
    #Create borders
    add_static_obstacle(space, (0,0), (5,500))
    add_static_obstacle(space, (0,0), (500,5))
    add_static_obstacle(space, (495,0), (5,500))
    add_static_obstacle(space, (0,495), (500,5))
    #Consider map as a 25x25 matrix using const size obstacles
    obstacle_size = (20,20)
    for y in range(25):
        prob_no = (7+np.random.choice(3, p = [0.1,0.1,0.8]))/10
        map_x = np.random.choice(2, size = 25, p = [prob_no, 1-prob_no])
        for x in range(25):
            if map_x[x]:
                add_static_obstacle(space, (25*x,25*y), obstacle_size)

    return 0

def generate_test_map(space):
    '''
    Function to generate a predefined map
    Arguments:
        space   : pymunk space object
    '''
    #Create borders
    add_static_obstacle(space, (0,0), (5,500))
    add_static_obstacle(space, (0,0), (500,5))
    add_static_obstacle(space, (495,0), (5,500))
    add_static_obstacle(space, (0,495), (500,5))
    #Create random obstacles
    add_static_obstacle(space, (90,5), (20,400))
    add_static_obstacle(space, (90,400), (90,20))
    add_static_obstacle(space, (380,220), (20,300))
    add_static_obstacle(space, (250,80), (20,350))
    add_static_obstacle(space, (250,80), (150,20))

def conv2matrix(screen, space, draw_options):
    '''
    Function to convert pygame window to numpy matrix
    Arguments:
        screen          : pygame screen object
        space           : pymunk space object
        draw_options    : pymunk draw options
    Returns:
        window_matrix   : numpy matrix of size screen_size
    '''
    screen.fill((0,0,0))
    space.debug_draw(draw_options)
    window_matrix = np.array(pygame.surfarray.array3d(screen))
    window_matrix = window_matrix[..., ::-1]
    window_matrix = np.flip(np.rot90(window_matrix, 1),0)
    return window_matrix

def draw_goal(screen, screen_size, goal, index):
    '''
    Function to draw each agent's goal position
    Arguments:
        screen          : pygame screen object
        screen_size     : (size_x, size_y)
        goal            : goal position
        index           : robot's index for color
    '''
    pygame.draw.line(screen, pygame.color.THECOLORS[colors[index]],
                     (goal[0]-10, screen_size[1]-goal[1]-10),
                     (goal[0]+10, screen_size[1]-goal[1]+10),
                     5)
    pygame.draw.line(screen, pygame.color.THECOLORS[colors[index]],
                     (goal[0]-10, screen_size[1]-goal[1]+10),
                     (goal[0]+10, screen_size[1]-goal[1]-10),
                     5)


class Robot(Thread):
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
        self.mrc_flags = 0b00000001                         #Byte Flag - MRC control

    def set_goal(self, goal):
        '''
        Set the goal parameter
        '''
        self.goal = goal

    def draw_goal(self, screen):
        '''
        Function to draw each agent's goal position
        Arguments:
            screen          : pygame screen object
        '''
        pygame.draw.line(screen, pygame.color.THECOLORS[colors[self.index]],
                         (self.goal[0]-10, screen_size[1]-self.goal[1]-10),
                         (self.goal[0]+10, screen_size[1]-self.goal[1]+10),
                         5)
        pygame.draw.line(screen, pygame.color.THECOLORS[colors[self.index]],
                         (self.goal[0]-10, screen_size[1]-self.goal[1]+10),
                         (self.goal[0]+10, screen_size[1]-self.goal[1]-10),
                         5)

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
        print("LOG: Got Robot",self.index, "Motion Plan")
        #Each time empty the paths
        self.act_pathx = []
        self.act_pathy = []
        #First check if path wasn't found
        for i in range(0,len(data.data)//2):
            #Following the list from normal simulator
            #Adjustments for Pymunk
            self.act_pathx.insert(0, data.data[i*2])
            self.act_pathy.insert(0, screen_size[1] - data.data[i*2+1])
            # self.robot_plan.append((data.data[i*2], data.data[i*2+1]))
        self.got_motion_plan = True     #Flip flag

    def holo_move_robot(self, vel):
        '''
        Function for holonomic control
        '''
        self.body.velocity = vel

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
        print("LOG: Robot", self.index, "Starting motion")
        #Get current position
        (curr_x, curr_y) = self.get_pos()
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #If LSB == 1 : Wait
            while self.mrc_flag & 0b00000001  == 1:
                self.holo_move_robot((0,0))
                time.sleep(1)
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
        print("LOG: Robot", self.index, "Goal reached")

    def run(self):
        '''
        Thread run
        '''
        #Wait until motion plan is received
        while not self.got_motion_plan:
            time.sleep(1)
            continue
        #If path wasn't found:
        if self.act_pathx[0] == 99999:
            print("LOG: Robot", self.index, "No Path Found! Stopping!")
            self.robot_reached = True   #Flip flag
            return 0
        #Otherwise, start controller
        self.basic_robot_controller()
