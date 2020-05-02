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

#Global variables
screen_size = (500,500)

#MoPAT files
import config_space
from MOPAT_astar import Astar

#Add a box shaped robot
def add_robot(space, pos, col):
    '''
    Function to generate bots
    Arguments:
        space   : pymunk space object
        pos     : (posx, posy) tuple
        col     : string color name
    Returns:
        shape   : robot shape
    '''
    #Create robot main body
    body = pymunk.Body(1, pymunk.moment_for_box(1, (20,20)))
    #Set body properties
    body.position = pos
    body.elasticity = 0
    body.friction = 1
    #Create shape/collision hull
    shape = pymunk.Poly.create_box(body, (20,20), radius = 0.5)
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

#Agent class
class Agent:
    #Create the robot shape
    def __init__(self, index, space, pos):
        self.body = add_robot(space, pos, "blue")
        self.index = index
        self.init_pos = pos
        self.config_space_generated = False
    #Get body data
    def get_body(self):
        return self.body
    #Holonomic motion control
    def move_robot(self, vel):
        self.body.velocity = vel
    #Robot manual controller
    # def man_controller(self, index):
    #     #Controls position
    #     print("Man_controller running")
    #     for event in pygame.event.get():
    #         print("event")
    #Robot config space thread
    def start_config_space_thread(self, window_matrix):
        self.map = window_matrix
        config_space_thread = Thread(target = self.static_config_space,
                                     args=())
        config_space_thread.daemon = True
        config_space_thread.start()
    #Robot config space generator
    def static_config_space(self):
        #Threshold map to get obstacle map
        _, obstacle_map = cv2.threshold(self.map[:,:,1], 200, 255, cv2.THRESH_BINARY)   #Using G channel
        # cv2.imshow("Obstacle map", obstacle_map)
        obstacle_map = obstacle_map.astype(bool)
        print("LOG: Generating configuration space")
        self.static_config = config_space.gen_config(obstacle_map, 12)
        print("LOG: Configuration space generated!")
        plt.matshow(self.static_config)
        self.config_space_generated = True
        plt.show()
    #Robot motion planning thread
    def start_motion_plan_thred(self,goal):
        self.goal = goal
        motion_plan_thread = Thread(target = self.motion_plan,
                                    args = ())
        motion_plan_thread.daemon = True
        motion_plan_thread.start()

    #Robot motion planning
    def motion_plan(self):
        global screen_size
        #Don't start motion planning until config space is generated
        while not self.config_space_generated:
            continue
        print("LOG: Starting motion planning")
        self.astar_obj = Astar(self.static_config, 0,0,screen_size[0], screen_size[1])
        self.pathx, self.pathy = self.astar_obj.find_best_route(screen_size[1]-self.init_pos[1],
                                            self.init_pos[0],
                                            screen_size[1]-self.goal[1],
                                            self.goal[0])
        plt.plot(self.pathx, self.pathy, "r")
        # plt.show()

#Simulator main control
def simulation():
    '''
    Main function to run the simulation
    '''
    global screen_size
    #Game initialization
    pygame.init()
    pygame.display.set_caption("MoPAT Pymunk Simulator Mk 1")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    #Create space
    space = pymunk.Space()
    #Create agent objects
    robot1 = Agent(1, space, (50,50))
    body = robot1.get_body()
    #Create map
    generate_test_map(space)

    #Control section
    #1. Generate config map
    screen.fill((0,0,0))
    space.debug_draw(draw_options)
    window_matrix = np.array(pygame.surfarray.array3d(screen))
    window_matrix = window_matrix[..., ::-1]
    window_matrix = np.flip(np.rot90(window_matrix, 1),0)
    robot1.start_config_space_thread(window_matrix)
    robot1.start_motion_plan_thred((50,50))
    #Simulator loop
    while True:
        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                print("LOG: Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("LOG: Exiting simulation")
                sys.exit(0)
            if event.type == KEYDOWN:
                if event.key == K_w:
                    print("LOG: W pressed")
                    robot1.move_robot((-100*np.sin(body.angle),100*np.cos(body.angle)))
                elif event.key == K_s:
                    print("LOG: S pressed")
                    robot1.move_robot((100*np.sin(body.angle),-100*np.cos(body.angle)))
                elif event.key == K_a:
                    print("LOG: A pressed")
                    body.angular_velocity = 1
                elif event.key == K_d:
                    print("LOG: D pressed")
                    body.angular_velocity = -1
            else:
                body.velocity = (0,0)
                body.angular_velocity = 0

        #Run controller in the simulation loop
        # robot1.man_controller(1)

        #Update screen
        screen.fill((0,0,0))
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    simulation()
