#MoPAT Library functions and classes
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import numpy as np
from threading import Thread

screen_size = (500,500)
#Colors
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
    body = pymunk.Body(1, pymunk.moment_for_circle(1, 0, 12))
    #Set body properties
    body.position = pos
    body.elasticity = 0
    body.friction = 1
    #Create shape/collision hull
    shape = pymunk.Circle(body, 15)
    shape.color = pygame.color.THECOLORS[col]
    #Show heading side
    heading = pymunk.Circle(body, 2, offset = (0,5))
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

def generate_random_map():
    '''
    Function to generate random maps for simulations
    '''
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
        index           : robot predefined index
        init_pos        : starting location
        goal            : goal location
        map             : the screen matrix
    '''
    def __init__(self, index, space, pos):
        '''
        Well, initialize
        '''
        super(Robot, self).__init__()
        self.shape = add_robot(space, pos, colors[index])
        self.body = self.shape.body
        self.index = index
        self.init_pos = pos
        self.screen_size = screen_size
        self.act_pathx = []
        self.act_pathy = []
        self.got_motion_plan = False
        self.robot_reached = 0

    def set_goal(self, goal):
        '''
        Set the goal paramter
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
        Arguments:
            data    :   ROS std_msgs/UInt32MultiArray
        '''
        print("Got Robot",self.index, "Motion Plan")
        #First check if path wasn't found
        for i in range(0,len(data.data)//2):
            #Following the list from normal simulator
            #Adjustments for Pymunk
            self.act_pathx.insert(0, data.data[i*2])
            self.act_pathy.insert(0, screen_size[1] - data.data[i*2+1])
            # self.robot_plan.append((data.data[i*2], data.data[i*2+1]))
        self.got_motion_plan = True

    def holo_move_robot(self, vel):
        '''
        Function for holonomic control
        '''
        self.body.velocity = vel

    def basic_robot_controller(self):
        '''
        Basic holonomic robot controller function
        '''
        #Constant velocity, angle controlled
        print("LOG: Robot", self.index, "Starting motion")
        #Get angle
        (curr_x, curr_y) = self.get_pos()
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #atan2 to get angle
            head_angle = np.arctan2(y-curr_y,x-curr_x)
            #set velocity
            self.holo_move_robot((80*np.cos(head_angle),
                             80*np.sin(head_angle)))
            #Update global positions
            # (curr_x, curr_y) =
            #wait until robot reaches the selected point
            while not ((int(x-curr_x) == 0) and (int(y-curr_y) == 0)):
                #update loc until reached
                (curr_x, curr_y) = self.get_pos()
                continue
        #If goal reached, stop
        self.holo_move_robot((0,0))
        print("LOG: Robot", self.index, "Goal reached")
        self.robot_reached = 1

    def run(self):
        '''
        Thread run
        '''
        #Wait until motion plan is found
        while not self.got_motion_plan:
            continue
        #If path wasn't found:
        if self.act_pathx[0] == 99999:
            print("Robot", self.index, "No Path Found! Stopping!")
            return 0
        self.basic_robot_controller()
