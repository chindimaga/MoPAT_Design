#MoPAT Library functions and classes
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import numpy as np

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

def
