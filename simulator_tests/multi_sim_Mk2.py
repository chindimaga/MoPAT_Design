#Guining Pertin - Started - 05-05-20
#Multi Robot Simulator

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
import itertools
import os

#Global variables
screen_size = (500,500)
config_space_generated = False
no_robots = 0
reached_robots = 0
#12 Robots - 12 Colors
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

#Config space primers
static_config = np.zeros(screen_size)
obstacle_map = np.zeros(screen_size)
map = np.zeros(screen_size)

#Multi-Robot Coodinator variables
#Index : (plan_generated, reached_goal)
mrc_input_flags = {}
#Index : (wait_robot)
mrc_output_flags = {}
mrc_start_motion = False
robot_paths = {}
robot_positions = {}

#MoPAT files
import config_space
from MOPAT_astar import Astar

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

def start_config_space_thread(screen, space, draw_options):
    '''
    Function to start the configuration space generation thread
    '''
    global map
    map = conv2matrix(screen, space, draw_options)
    config_space_thread = Thread(target = static_config_space, args = ())
    config_space_thread.daemon = True
    config_space_thread.start()

def static_config_space():
    '''
    Function to generate static configuration space
    '''
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

def plot_all():
    '''
    Function to plot, almost everything needed
    '''
    global no_robots
    #Set plots
    fig, (ax1,ax2) = plt.subplots(1,2)
    ax1.title.set_text("Static Configuration Space")
    ax1.axes.get_xaxis().set_visible(False)
    ax1.axes.get_yaxis().set_visible(False)
    ax2.title.set_text("A* Algorithm Plan")
    ax2.axes.get_xaxis().set_visible(False)
    ax2.axes.get_yaxis().set_visible(False)
    mngr = plt.get_current_fig_manager()
    mngr.window.wm_geometry("+600+100")
    #Plot config and map
    ax1.matshow(static_config)
    ax2.imshow(obstacle_map)
    plt.show(block=False)
    plt.pause(0.5)
    #Wait for mrc_signal
    while not mrc_start_motion:
        continue
    #Plot each
    print("LOG: Plotting all paths")
    for i in robot_paths:
        #Plot the paths
        ax1.plot(robot_paths[i][0], robot_paths[i][1], colors[i])
        ax2.plot(robot_paths[i][0], robot_paths[i][1], colors[i])
    plt.show()

def start_coordinator_thread():
    '''
    Function to start the multi-robot coordinator thread
    '''
    coordinator_thread = Thread(target = multi_robot_coordinator, args = ())
    coordinator_thread.daemon = True
    coordinator_thread.start()

def multi_robot_coordinator():
    '''
    Function that handles all the robots initialization and collision
    '''
    global mrc_input_flags
    global mrc_output_flags
    global mrc_start_motion
    global robot_positions
    started_all = False
    #Check it continuously
    while True:
        #1. Motion coordinator - start everyone at the same time
        #Motion Coordinator check
        motion_plans_generated = 0
        if not started_all:
            for i in mrc_input_flags:
                #Update number of motion plans generated
                if mrc_input_flags[i][0]: motion_plans_generated += 1
            #Check if all plans generated
            if motion_plans_generated == no_robots:
                mrc_start_motion = True
                started_all = True
        #2. Robot-Robot collision control
        #Renew the collision check data
        for i in mrc_output_flags:
            mrc_output_flags[i][0] = False
        #Check collision between each combination
        for x in itertools.combinations(robot_positions, 2):
            #If near,
            if check_robot_collision(x[0], x[1]):
                #One with lower index waits
                mrc_output_flags[min(x)][0] = mrc_output_flags[min(x)][0] or True
                #Otherwise none waits
            else:
                mrc_output_flags[x[0]][0] = mrc_output_flags[x[0]][0] or False
                mrc_output_flags[x[1]][0] = mrc_output_flags[x[1]][0] or False
        # print(robot_positions[0])
        time.sleep(0.1)
    #3. Goal distance coordinator
    #4. Formation control coordinator
    return 0

def check_robot_collision(i, j):
    '''
    Function to check collision between two robots
    '''
    #Check distance between two robots
    x = robot_positions[i][0] - robot_positions[j][0]
    y = robot_positions[i][1] - robot_positions[j][1]
    #If less than minimum, nearby
    if np.linalg.norm([x,y]) < 100:
        return 1
    #If no one nearby
    return 0

class Agent(Thread):
    '''
    The robot agent class
    Parameters:
        body            : robot body object
        index           : robot predefined index
        init_pos        : starting location
        goal            : goal location
        map             : the screen matrix
    '''
    def __init__(self, index, space, pos, goal):
        '''
        Well, initialize
        '''
        super(Agent, self).__init__()
        global no_robots
        global mrc_input_flags
        global mrc_output_flags
        no_robots += 1
        self.shape = add_robot(space, pos, colors[index])
        self.body = self.shape.body
        self.index = index
        self.init_pos = pos
        self.goal = goal
        robot_positions[self.index] = (pos[0], pos[1])
        mrc_input_flags[index] = [False, False]
        mrc_output_flags[index] = [False, False]

    def holo_move_robot(self, vel):
        '''
        Function for holonomic control
        '''
        self.body.velocity = vel

    def draw_goal(self, screen):
        '''
        Function to draw each agent's goal position
        '''
        pygame.draw.line(screen, pygame.color.THECOLORS[colors[self.index]],
                         (self.goal[0]-10, screen_size[1]-self.goal[1]-10),
                         (self.goal[0]+10, screen_size[1]-self.goal[1]+10),
                         5)
        pygame.draw.line(screen, pygame.color.THECOLORS[colors[self.index]],
                         (self.goal[0]-10, screen_size[1]-self.goal[1]+10),
                         (self.goal[0]+10, screen_size[1]-self.goal[1]-10),
                         5)

    def motion_plan(self):
        '''
        Function to generate agent's motion plan using A*
        '''
        global static_config
        global robot_paths
        global mrc_input_flags
        print("LOG: Robot_", self.index,"Starting motion planning")
        self.astar_obj = Astar(self.index, static_config, 0,0,screen_size[0], screen_size[1])
        self.gen_pathy, self.gen_pathx = self.astar_obj.find_best_route(screen_size[1]-self.init_pos[1],
                                            self.init_pos[0],
                                            screen_size[1]-self.goal[1],
                                            self.goal[0])
        #Store to global variable
        robot_paths[self.index] = (self.gen_pathx, self.gen_pathy)
        #Adjustments for pymunk
        self.act_pathx = np.array(self.gen_pathx[::-1])
        self.act_pathy = screen_size[1] - np.array(self.gen_pathy[::-1])
        #Set mrc flag
        mrc_input_flags[self.index][0] = True

    def robot_move(self):
        '''
        Basic robot controller function
        '''
        global reached_robots
        global robot_positions
        #Constant velocity, angle controlled
        print("LOG: Robot_", self.index, " Starting motion")
        #Get angle
        curr_x = self.act_pathx[0]
        curr_y = self.act_pathy[0]
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #First check for mrc control signal
            while mrc_output_flags[self.index][0]:
                self.holo_move_robot((0,0))
            #atan2 to get angle
            head_angle = np.arctan2(y-curr_y,x-curr_x)
            #set velocity
            self.holo_move_robot((80*np.cos(head_angle),
                             80*np.sin(head_angle)))
            #set curr loc
            curr_x = self.body.position[0]
            curr_y = self.body.position[1]
            #Update global positions
            robot_positions[self.index] = (curr_x, curr_y)
            #wait until robot reaches the selected point
            while not ((int(x-curr_x) == 0) and (int(y-curr_y) == 0)):
                #update loc until reached
                curr_x = self.body.position[0]
                curr_y = self.body.position[1]
                robot_positions[self.index] = (curr_x, curr_y)
                continue
        #If goal reached, stop
        self.holo_move_robot((0,0))
        print("LOG: Robot_", self.index, " Goal reached")
        #Reset the advertised position after completion
        robot_positions[self.index] = (0,0)
        mrc_input_flags[self.index][1] = True
        reached_robots += 1

    def run(self):
        '''
        Thread run
        '''
        #Don't start motion planning until config_space is generated
        while not config_space_generated:
            continue
        #Get motion plan
        self.motion_plan()
        #Don't start motion control until mrc signal
        while not mrc_start_motion:
            continue
        #Move robot
        self.robot_move()

def simulation():
    '''
    Main function to run the simulation
    '''
    global screen_size
    global no_robots
    global reached_robots
    #Game initialization
    os.environ['SDL_VIDEO_WINDOW_POS'] = "+100,+100"
    pygame.init()
    pygame.display.set_caption("Starting MoPAT Multi-Robot Simulator Mk 1")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    goal = (450,300)        #Pymunk coords
    #Create space
    space = pymunk.Space()
    #Create agent object
    robot0 = Agent(0, space, (200,200), (450,300))
    robot1 = Agent(1, space, (50,100), (450,100))
    robot2 = Agent(2, space, (200,50), (450,180))
    #Create map
    generate_test_map(space)
    #Start config space thread
    start_config_space_thread(screen, space, draw_options)
    #Start multi_robot_coodinator thread
    start_coordinator_thread()
    #Start robot global controller thread
    robot0.start()
    robot1.start()
    robot2.start()
    # Simulator graphic loop
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
            print("LOG: Simulation completed")
            time.sleep(5)
            sys.exit(0)
        #Update screen
        screen.fill((0,0,0))
        # pygame.draw.rect(screen, pygame.color.THECOLORS["yellow"],
        #                  (goal[0]-10, screen_size[1]-goal[1]-10,20,20))
        robot0.draw_goal(screen)
        robot1.draw_goal(screen)
        robot2.draw_goal(screen)
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    sys.exit(simulation())
