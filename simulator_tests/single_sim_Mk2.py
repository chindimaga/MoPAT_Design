#Guining Pertin - Started - 30-04-20
#Test Pymunk physics simulator

#Import the required libraries
import sys
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import numpy as np
import cv2
import matplotlib.pyplot as plt
from threading import Thread
import time

#Global variables
screen_size = (500,500)
goal = (450,300)        #Pymunk coords
config_space_generated = False
no_robots = 0
reached_robots = 0

#Config space primers
static_config = np.zeros(screen_size)
obstacle_map = np.zeros(screen_size)
map = np.zeros(screen_size)

#Robot paths
robot_paths = {}
#12 Robots - 12 Colors
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

#MoPAT files
import config_space
from MOPAT_astar import Astar

#Add a circle shaped robot
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
    return body

#Add a box shaped static obstacle
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

#Generate random map
def generate_random_map():
    return 0

#Generate test map
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

#Convert pygame screen to matrix
def conv2matrix(screen, space, draw_options):
    screen.fill((0,0,0))
    space.debug_draw(draw_options)
    window_matrix = np.array(pygame.surfarray.array3d(screen))
    window_matrix = window_matrix[..., ::-1]
    window_matrix = np.flip(np.rot90(window_matrix, 1),0)
    return window_matrix

#Start configuration space thread
def start_config_space_thread(screen, space, draw_options):
    global map
    map = conv2matrix(screen, space, draw_options)
    config_space_thread = Thread(target = static_config_space, args = ())
    config_space_thread.daemon = True
    config_space_thread.start()

#Robot config space generator
def static_config_space():
    global obstacle_map
    global static_config
    global config_space_generated
    #Threshold map to get obstacle map
    _, obstacle_map = cv2.threshold(cv2.cvtColor(map, cv2.COLOR_BGR2GRAY), 250, 255, cv2.THRESH_BINARY)   #Using G channel
    obstacle_map = obstacle_map.astype(bool)
    print("LOG: Generating configuration space")
    static_config = config_space.gen_config(obstacle_map, 20)
    print("LOG: Configuration space generated!")
    config_space_generated = True
    plot_all()

#Plot plot plot!
def plot_all():
    global no_robots
    #Set plots
    fig, (ax1,ax2) = plt.subplots(1,2)
    ax1.title.set_text("Static Configuration Space")
    ax1.axes.get_xaxis().set_visible(False)
    ax1.axes.get_yaxis().set_visible(False)
    ax2.title.set_text("A* Algorithm Plan")
    ax2.axes.get_xaxis().set_visible(False)
    ax2.axes.get_yaxis().set_visible(False)
    #Plot config and map
    ax1.matshow(static_config)
    ax2.imshow(obstacle_map)
    plt.show(block=False)
    plt.pause(1)
    #If all paths not generated yet
    while len(robot_paths) != no_robots:
        continue
    #Plot each
    print("LOG: Plotting all paths")
    for i in robot_paths:
        #Plot the paths
        ax1.plot(robot_paths[i][0], robot_paths[i][1], colors[i])
        ax2.plot(robot_paths[i][0], robot_paths[i][1], colors[i])
    plt.show()

#Agent class
class Agent:
    '''
    The robot agent class
    Parameters:
        body            : robot body object
        index           : robot predefined index
        init_pos        : starting location
        goal            : goal location
        map             : the screen matrix
    '''
    #Create the robot shape
    def __init__(self, index, space, pos):
        global no_robots
        no_robots += 1
        self.body = add_robot(space, pos, colors[index])
        self.index = index
        self.init_pos = pos
        self.motion_plan_generated = False

    #Get body data
    def get_body(self):
        return self.body

    #Holonomic motion control
    def move_robot(self, vel):
        self.body.velocity = vel

    #Combined robot_controller thread
    def start_global_controller_thread(self, goal):
        #Set parameters
        self.goal = goal
        #Controller thread
        global_controller_thread = Thread(target = self.global_controller, args = ())
        global_controller_thread.daemon = True
        global_controller_thread.start()

    #Combined global robot_controller
    def global_controller(self):
        #Get motion plan
        self.motion_plan()
        #Move robot
        self.robot_move()

    #Robot motion planning
    def motion_plan(self):
        global screen_size
        global static_config
        global robot_paths
        #Don't start motion planning until config space is generated
        while not config_space_generated:
            continue
        print("LOG: Robot_", self.index,"Starting motion planning")
        self.astar_obj = Astar(self.index, static_config, 0,0,screen_size[0], screen_size[1])
        self.gen_pathy, self.gen_pathx = self.astar_obj.find_best_route(screen_size[1]-self.init_pos[1],
                                            self.init_pos[0],
                                            screen_size[1]-self.goal[1],
                                            self.goal[0])
        self.motion_plan_generated = True
        #Store to global variable
        robot_paths[self.index] = (self.gen_pathx, self.gen_pathy)
        #Adjustments for pymunk
        self.act_pathx = np.array(self.gen_pathx[::-1])
        self.act_pathy = screen_size[1] - np.array(self.gen_pathy[::-1])

    #Robot movement
    def robot_move(self):
        global reached_robots
        #Won't start until motion plan
        while not self.motion_plan_generated:
            continue
        print("LOG: Robot_", self.index, " Starting motion")
        #Constant velocity, angle controlled
        #Get angle
        curr_x = self.act_pathx[0]
        curr_y = self.act_pathy[0]
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #atan2 to get angle
            head_angle = np.arctan2(y-curr_y,x-curr_x)
            #set velocity
            self.move_robot((100*np.cos(head_angle),
                             100*np.sin(head_angle)))
            #set curr loc
            curr_x = self.body.position[0]
            curr_y = self.body.position[1]
            #wait until robot reaches the selected point
            while not ((int(x-curr_x) == 0) and (int(y-curr_y) == 0)):
                #update loc until reached
                curr_x = self.body.position[0]
                curr_y = self.body.position[1]
                continue
        #If goal reached, stop
        self.move_robot((0,0))
        print("LOG: Robot_", self.index, " Goal reached")
        reached_robots += 1

#Simulator main control
def simulation():
    '''
    Main function to run the simulation
    '''
    global screen_size
    global no_robots
    global reached_robots
    #Game initialization
    pygame.init()
    pygame.display.set_caption("Starting MoPAT Pymunk Simulator Mk 1")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    #Create space
    space = pymunk.Space()
    #Create agent object
    robot0 = Agent(0, space, (50,50))
    #Create map
    generate_test_map(space)
    #Start config space thread
    start_config_space_thread(screen, space, draw_options)
    #Start robot global controller
    robot0.start_global_controller_thread(goal)
    #Simulator graphic loop
    while True:
        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                print("LOG: Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("LOG: Exiting simulation")
                sys.exit(0)
        if reached_robots == no_robots:
            print("LOG: All robots reached")
            print("LOG: Exiting simulation in 5s")
            time.sleep(5)
            sys.exit(0)
        #Update screen
        screen.fill((0,0,0))
        pygame.draw.rect(screen, pygame.color.THECOLORS["yellow"],
                         (goal[0]-10, screen_size[1]-goal[1]-10,20,20))
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    simulation()
