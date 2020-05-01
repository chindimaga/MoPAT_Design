#Guining Pertin - Started - 30-04-20
#Test Pymunk physics simulator

#Import the required libraries
import sys
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import numpy as np

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
    body = pymunk.Body(1, pymunk.moment_for_box(1, (50,50)))
    #Set body properties
    body.position = pos
    body.elasticity = 0
    body.friction = 1
    #Create shape/collision hull
    shape = pymunk.Poly.create_box(body, (50,50), radius = 3.0)
    shape.color = pygame.color.THECOLORS[col]
    #Show heading side
    heading = pymunk.Circle(body, 5, offset = (0,20))
    heading.color = pygame.color.THECOLORS["grey"]
    #Add the object
    space.add(body, shape, heading)
    return body

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
    body.position = pos[0] + size[0]/2, pos[1] + size[1]/2
    if body.position[0] < 0 or body.position[1] < 0: return 0
    #Create box shape
    shape = pymunk.Poly.create_box(body, size, 0.0)
    shape.color = pygame.color.THECOLORS["white"]
    #Add the object
    space.add(body, shape)

def move(body, gravity, damping, dt, vel):
    pymunk.Body.update_velocity(body, vel, damping, dt)

#Generate random map
def generate_random_map():
    return 0

#Generate test map
def generate_test_map(space):
    #Create borders
    add_static_obstacle(space, (0,0), (10,1000))
    add_static_obstacle(space, (0,0), (1000,10))
    add_static_obstacle(space, (990,0), (10,1000))
    add_static_obstacle(space, (0,990), (1000,10))
    #Create random obstacles
    add_static_obstacle(space, (150,10), (50,800))
    add_static_obstacle(space, (150,810), (150,50))
    add_static_obstacle(space, (750,500), (50,500))
    add_static_obstacle(space, (500,200), (50,700))
    add_static_obstacle(space, (500,150), (300,50))

#Agent class
class Agent:
    #Create the robot shape
    def __init__(self, index, space, pos):
        self.body = add_robot(space, pos, "red")
    #Get body data
    def get_body(self):
        return self.body
    #Robot manual controller
    def man_controller(self, index):
        #Controls position
        print("Man_controller running")
        for event in pygame.event.get():
            print("event")


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

    #Create agent objects
    robot1 = Agent(1, space, (80,80))
    body = robot1.get_body()
    #Add an obstacle
    generate_test_map(space)

    #Simulator loop
    while True:
        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                print("Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("Exiting simulation")
                sys.exit(0)
            if event.type == KEYDOWN:
                if event.key == K_w:
                    print("W pressed")
                    body.velocity = (-100*np.sin(body.angle),100*np.cos(body.angle))
                elif event.key == K_s:
                    print("S pressed")
                    body.velocity = (100*np.sin(body.angle),-100*np.cos(body.angle))
                elif event.key == K_a:
                    print("A pressed")
                    body.angular_velocity = 1
                elif event.key == K_d:
                    print("D pressed")
                    body.angular_velocity = -1
            else:
                body.velocity = (0,0)
                body.angular_velocity = 0

        #Run controller in the simulation loop
        # robot1.man_controller(1)

        #Show the space
        screen.fill((0,0,0))                        #background - black
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    sys.exit(simulation())
