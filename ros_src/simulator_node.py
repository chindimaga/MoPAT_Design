#Guining Pertin - 14-05-20
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

#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

screen_size = (500,500)
steps = 50
bridge = CvBridge()

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

def simulator_node():
    '''
    Function to run the simulation
    '''
    global screen_size
    #Game initialization
    os.environ['SDL_VIDEO_WINDOW_POS'] = "+100,+100"
    pygame.init()
    pygame.display.set_caption("Starting MoPAT Multi-Robot Simulator Mk 1")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    #Create space
    space = pymunk.Space(threaded = True)
    space.threads = 2
    #Create node
    rospy.init_node("simualtor_node", anonymous = True)
    #Publish simulator raw output
    pub = rospy.Publisher("mopat/sim_raw_image", Image, queue_size=5)
    #Set rate
    rate = rospy.Rate(30)
    #Create map
    generate_test_map(space)
    #Run the simulator
    while not rospy.is_shutdown():

        #Exiting the simulator
        for event in pygame.event.get():
            if event.type == QUIT:
                print("LOG: Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("LOG: Exiting simulation")
                sys.exit(0)
        #Update screen
        screen.fill((0,0,0))
        space.step(1/steps)
        space.debug_draw(draw_options)
        #Get raw map
        map = conv2matrix(screen, space, draw_options)
        #Publish raw data
        pub.publish(bridge.cv2_to_imgmsg(map, encoding="passthrough"))
        pygame.display.flip()
        clock.tick(steps)
        # print(clock.get_fps())

if __name__ == "__main__":
    try:
        simulator_node()
    except rospy.ROSInterruptException:
        pass
