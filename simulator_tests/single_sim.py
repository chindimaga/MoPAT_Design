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
goal = (450,450)        #Pymunk coords

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
        shape   : robot shape
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
        self.motion_plan_generated = False

    #Get body data
    def get_body(self):
        return self.body

    #Holonomic motion control
    def move_robot(self, vel):
        self.body.velocity = vel

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
        self.obstacle_map = obstacle_map.astype(bool)
        print("LOG: Generating configuration space")
        self.static_config = config_space.gen_config(self.obstacle_map, 20)
        print("LOG: Configuration space generated!")
        self.config_space_generated = True
        # plt.show()

    #Robot motion planning thread
    def start_motion_plan_thread(self,goal):
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
        self.gen_pathy, self.gen_pathx = self.astar_obj.find_best_route(screen_size[0]-self.init_pos[0],
                                            self.init_pos[1],
                                            screen_size[0]-self.goal[0],
                                            self.goal[1])
        self.motion_plan_generated = True
        # print(self.gen_pathx)
        # print(self.gen_pathy)
        #Adjustments for pymunk
        self.act_pathx = np.array(self.gen_pathx[::-1])
        self.act_pathy = screen_size[1] - np.array(self.gen_pathy[::-1])
        # print(self.act_pathx)
        # print(self.act_pathy)

    #Plotting thread
    def start_plot_all_thread(self):
        plot_all_thread = Thread(target = self.plot_all, args = ())
        plot_all_thread.daemon = True
        plot_all_thread.start()

    #Plot plot plot!
    def plot_all(self):
        fig, (ax1,ax2) = plt.subplots(1,2)
        ax1.title.set_text("Static Configuration Space")
        ax1.axes.get_xaxis().set_visible(False)
        ax1.axes.get_yaxis().set_visible(False)
        ax2.title.set_text("A* Algorithm Plan")
        ax2.axes.get_xaxis().set_visible(False)
        ax2.axes.get_yaxis().set_visible(False)
        while not self.config_space_generated:
            continue
        ax1.matshow(self.static_config)
        ax2.imshow(self.obstacle_map)
        plt.show(block=False)
        plt.pause(1)
        while not self.motion_plan_generated:
            continue
        ax1.plot(self.gen_pathx, self.gen_pathy, "r")
        ax2.plot(self.gen_pathx, self.gen_pathy, "r")
        plt.show()

    #Robot movement thread
    def start_robot_move_thread(self):
        robot_move_thread = Thread(target = self.robot_move,
                                    args = ())
        robot_move_thread.daemon = True
        robot_move_thread.start()

    #Robot movement
    def robot_move(self):
        #Won't start until motion plan
        while not self.motion_plan_generated:
            continue
        print("LOG: Starting Motion")
        #Constant velocity, angle controlled
        #Get angle
        curr_x = self.act_pathx[0]
        curr_y = self.act_pathy[0]
        for (x,y) in zip(self.act_pathx[1:], self.act_pathy[1:]):
            #atan2 to get angle
            head_angle = np.arctan2(y-curr_y,x-curr_x)
            # print("Y: ", y-curr_y, "\tX: ", x-curr_x,
            #       "\tAng: ", head_angle*180/np.pi)
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
        print("LOG: Motion Completed")

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
    #Create agent object
    robot = Agent(1, space, (50,50))
    body = robot.get_body()
    #Create map
    generate_test_map(space)

    #Control section
    #1. Generate configuration space
    screen.fill((0,0,0))
    space.debug_draw(draw_options)
    window_matrix = np.array(pygame.surfarray.array3d(screen))
    window_matrix = window_matrix[..., ::-1]
    window_matrix = np.flip(np.rot90(window_matrix, 1),0)
    robot.start_config_space_thread(window_matrix)
    #2. Generate motion plan
    robot.start_motion_plan_thread(goal)
    #3. Plot stuff
    robot.start_plot_all_thread()
    #4. Run stuff
    robot.start_robot_move_thread()
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
                    robot.move_robot((0,100))
                    # print(body.angle*360/np.pi)
                elif event.key == K_s:
                    print("LOG: S pressed")
                    robot.move_robot((0,-100))
                elif event.key == K_a:
                    print("LOG: A pressed")
                    robot.move_robot((-100,0))
                elif event.key == K_d:
                    print("LOG: D pressed")
                    robot.move_robot((100,0))
            else:
                body.velocity = (0,0)
                body.angular_velocity = 0

        #Run controller in the simulation loop
        # robot.man_controller(1)

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
