#Guining Pertin - Started - 29-04-20
#Test Pymunk physics simulator

#Import the required libraries
import sys
import pygame
from pygame.locals import *
import pymunk
from pymunk import pygame_util
import random

#Funtion to create a ball
def add_ball(space):
    mass = 1
    radius = 20
    #Set the moment of inertial for a ball - predefined for ball
    moment = pymunk.moment_for_circle(mass, 0, radius)
    #Create ball body using mass and moment
    body = pymunk.Body(mass, moment)
    #Set a position
    x = random.randint(120,380)
    body.position = x, 550
    #Create the collision mask basically called shape here
    shape = pymunk.Circle(body, radius)
    #Add the body and its shape to the simulation
    space.add(body, shape)
    return shape

#To draw balls you can use a manual drawing function or use pygame draw_ball
#Here we will use pymunk's debug_draw

#Function to create the static ground
def add_static_L(space):
    #To create static bodies - never add it to space like
    #dynamic bodies - set my setting the body type
    body = pymunk.Body(body_type = pymunk.Body.STATIC)
    body.position = (250,250)
    #Create the lines
    l1 = pymunk.Segment(body, (-150,0), (100,0), 5)
    l2 = pymunk.Segment(body, (-150,0), (-150,50),5)
    #Add the segments into the space, not the body
    space.add(l1, l2)
    return l1,l2

#Function to create the dynamic ground
def add_L(space):
    #Create the rotation center body to act as a static point in the joint
    #so that the line can rotate around it. NO SHAPE/Collision hull added to it
    rotation_center_body = pymunk.Body(body_type = pymunk.Body.STATIC)
    rotation_center_body.position = (250,250)
    #Create a static body to limit the motion
    rotation_limit_body = pymunk.Body(body_type = pymunk.Body.STATIC)
    rotation_limit_body.position = (100,250)
    #The L shape to be added to the screen is not static anymore
    #mass 10 and random moment wrt the rotation center
    body = pymunk.Body(10,10000)
    body.position = (250,250)
    #Create the lines
    l1 = pymunk.Segment(body, (-150,0), (100,0), 5)
    l2 = pymunk.Segment(body, (-150,0), (-150,50),5)

    #Joint functions take in
    #1st body, 2nd body, 1st body joint location, 2nd_body joint location
    #Create a pinjoint between body and rotation_center_body
    rotation_center_joint = pymunk.PinJoint(body, rotation_center_body,
                                            (0,0), (0,0))
    #Create a slidejoint between body and rotation_center_body
    #joint_limit defines the min and max the body will slide wrt to each other
    joint_limit = 10
    rotation_limit_joint = pymunk.SlideJoint(body, rotation_limit_body,
                                             (-150,0), (0,0), 0, joint_limit)
    space.add(l1, l2, body, rotation_center_joint, rotation_limit_joint)
    return l1,l2

#Main function
def main():
    #Initilization of the game
    pygame.init()
    screen = pygame.display.set_mode((500,500))
    pygame.display.set_caption("Test 1")
    clock = pygame.time.Clock()

    #Create a space and set the gravity
    space = pymunk.Space()
    space.gravity = (0.0,-100.0)

    #Balls balls everywhere!
    balls = []
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    #Add the dynamic L
    lines = add_L(space)

    ticks_to_next_ball = 10

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)
        #Spawn ball again
        ticks_to_next_ball -= 1
        if ticks_to_next_ball <= 0:
            ticks_to_next_ball = 10
            ball_shape = add_ball(space)
            balls.append(ball_shape)

        #Remove balls slowly
        balls_to_remove = []
        for ball in balls:
            #If falls beyond a limit
            if ball.body.position.y < 200:
                balls_to_remove.append(ball)
        for ball in balls_to_remove:
            #remove the ball hull and the body
            #ball contains the draw/hull/shape
            #ball.body contains its body information
            space.remove(ball, ball.body)
            balls.remove(ball)
        #Step() function on ournspace to step the simulation one step fwd
        #Keep the step size constant ind of the framerate
        space.step(1/50.0)
        screen.fill((255,255,255))
        #Draw in the given space
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(50)

if __name__ == "__main__":
    sys.exit(main())
