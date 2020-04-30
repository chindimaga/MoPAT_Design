#Guining Pertin - Started - 30-04-20
#Test Pymunk physics simulator

#Import the required libraries
import sys
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util

#Add a box shaped robot
def add_robot(space, pos):
    '''
    Function to generate bots
    Arguments:
        space   : pymunk space object
        pos     : (posx, posy) tuple
    Returns:
        shape   : robot shape
    '''
    #Create robot body
    body = pymunk.Body(1, 200)                          #mass, moment
    #Set body properties
    body.position = pos
    body.elasticity = 0
    body.friction = 0.7
    #Create shape/collision hull
    shape = pymunk.Poly.create_box(body, (50,50), 0.0)  #body, size, border
    #Add the object
    space.add(body, shape)
    return shape

""" NEED TO ADD BOUNDARY CONDITIONS AND CENTER OFFSET"""
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
    body.position = pos
    #Create box shape
    shape = pymunk.Poly.create_box(body, size, 0.0)
    #Add the object
    space.add(body, shape)
    return shape

#Generate random map
def genrate_map():
    return 0

#Agent class
class Agent:
    #Create the robot shape
    def __init__(self, index, space, pos):
        self.shape = add_robot(space, pos)
    #Get shape data
    def get_shape(self):
        return self.shape
    #Robot manual controller
    #Robot config space generator
    #Robot motion planning
    #Robot motion controller
    #Robot motion tracker

#Simulator main control
def simulation():
    '''
    Main function to run the simulation
    '''
    #Game initialization
    pygame.init()
    pygame.display.set_caption("MoPAT Pymunk Simulator Mk 1")
    screen = pygame.display.set_mode((1000,1000))
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()

    #Create space
    space = pymunk.Space()
    space.gravity = (0.0,-100.0)
    #Generate a robot
    robot_shape = add_robot(space, (100,100))

    #Create agent objects
    robot1 = Agent(1, space, (500,500))
    robot2 = Agent(2, space, (300,500))
    robot3 = Agent(3, space, (700,500))
    draw_options.color_for_shape(robot1.get_shape())
    draw_options.shape_dynamic_color = (255,255,255,255)
    draw_options.color_for_shape(robot2.get_shape())
    draw_options.shape_dynamic_color = (0,255,255,255)
    #Add an obstacle
    # obstacle_shape = add_static_obstacle(space, (500,200), (500,50))

    #Simulator loop
    while True:
        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                sys.exit(0)

        #Show the space
        screen.fill((0,0,0))                        #background - black
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    sys.exit(simulation())
