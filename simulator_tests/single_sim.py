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
    space : pymunk space object
    posx  : (posx, posy) tuple
    '''
    #Create robot body
    body = pymunk.Body(1, 200)                          #mass, moment
    #Set body properties
    body.position = pos[0], pos[1]
    body.elasticity = 0
    body.friction = 0.7
    #Create shape/collision hull
    shape = pymunk.Poly.create_box(body, (50,50), 0.0)  #body, size, border
    #Add the space object
    space.add(body, shape)
    return shape

#Simulator main control
def simulation():
    #Game initialization
    pygame.init()
    pygame.display.set_caption("MoPAT Pymunk Simulator Mk 1")
    screen = pygame.display.set_mode((1000,1000))                              #background - black
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()

    #Create space
    space = pymunk.Space()

    #Generate a robot
    robot_shape = add_robot(space, (500,500))
    #Simulator loop
    while True:
        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)
        #Show the space
        screen.fill((0,0,0))
        space.step(1/50.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    sys.exit(simulation())
