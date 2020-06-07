#MoPAT Design Lab - Guining Pertin
#Simulator node - 12-05-20

'''
This node runs the entire simulation(only)
Subscribed topics:
    mopat/control/motion_plan_{i}       -   std_msgs/UInt32MultiArrays
    mopat/control/mrc_output_flags      -   std_msgs/ByteMultiArray
Published topics:
    mopat/tracking/raw_image            -   sensor_msgs/Image (BGR)
    mopat/robot/robot_starts            -   std_msgs/UInt32MultiArray
    mopat/robot/robot_goals             -   std_msgs/UInt32MultiArray
    mopat/robot/robot_positions         -   std_msgs/UInt32MultiArray
    mopat/robot/robot_num               -   std_msgs/UInt32
Work:
    Uses pymunk to run simulation and runs robot threads
'''

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray, UInt32, ByteMultiArray
#Others
import sys
import numpy as np
import os
#Mopat
from mopat_lib import *

#Global variables
robot_list = {}                  #Dict - robot_index : ith Robot object

def mrc_cb(data):
    '''
    Get MRC control signal
    Arguments:
        data    :   ROS sensor_msgs/ByteMultiArray
    '''
    global robot_list
    for i in range(len(data.data)):
        robot_list[i].mrc_flag = data.data[i]

def simulator_node():
    '''
    Function to run the simulation
    '''
    global robot_list
    #Local variables
    goals_multiarray = UInt32MultiArray()       #Robot goals - UInt32MultiArray type rosmsg
    positions_multiarray = UInt32MultiArray()   #Robot positions - UInt32MultiArray type rosmsg
    starts_multiarray = UInt32MultiArray()      #Robot starts - UInt32MultiArray type rosmsg
    robot_starts = {}                           #Dict - robot_index : robot start
    robot_goals = {}                            #Dict - robot_index : robot goal
    steps = 50                                  #Simulation steps
    robot_index = 0                             #Number of robots and indexing variable
    got_starts = False                          #Flag - True if user inputs all starts
    got_goals = False                           #Flag - True if user inputs all goals
    started = False                             #Flag - True if simulation started
    got_mouse_click = False                     #Flag - True if mouse clicked
    bridge = CvBridge()                         #Required for rosmsg-cv conversion
    #Initialize node
    rospy.init_node("simulator_node")
    print("LOG: Started MoPAT Multi-Robot Simulator MkII node")
    #Game initialization
    os.environ['SDL_VIDEO_WINDOW_POS'] = "+0,+50"   #Set position
    pygame.init()
    pygame.display.set_caption("MoPAT Multi-Robot Simulator MkII")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    space = pymunk.Space()                          #Game space
    #Subscriber and publishers
    rospy.Subscriber("/mopat/control/mrc_output_flags", ByteMultiArray, mrc_cb)
    pub_raw = rospy.Publisher("mopat/tracking/raw_image", Image, queue_size=5)
    pub_starts = rospy.Publisher("mopat/robot/robot_starts", UInt32MultiArray, queue_size=5)
    pub_goals = rospy.Publisher("mopat/robot/robot_goals", UInt32MultiArray, queue_size=5)
    pub_positions = rospy.Publisher("mopat/robot/robot_positions", UInt32MultiArray, queue_size=5)
    pub_num = rospy.Publisher("mopat/robot/robot_num", UInt32, queue_size=5)
    #Create map
    # generate_empty_map(space)
    # generate_test_map(space)
    generate_random_map(space)
    print("USER: Enter initial positions now")
    #Simulate!
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            #Exiting
            if event.type == QUIT:
                print("EXIT: Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("EXIT: Exiting simulation")
                sys.exit(0)
            #Get user input
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                mouse_y = screen_size[1] - mouse_y  #Pygame pymunk conversion
                got_mouse_click = True              #Flip flag
            if (event.type == KEYDOWN) and (event.key == K_w) and not got_goals:
                print("USER: Enter goals now")
                robot_index = 0                     #Reset index for goals
                goals_multiarray.data.clear()       #Clear data for goals
                got_starts = True                   #Flip flag
        #Don't run if all goals not found yet
        if not got_goals:
            #If mouse click found
            if got_mouse_click:
                #If user inputs starting locations
                if not got_starts:
                    print("LOG: Got Robot", robot_index,
                          "Start:", mouse_x,";",mouse_y)
                    #Create robot object with start
                    robot_list[robot_index] = Robot(robot_index, space, (mouse_x, mouse_y))
                    starts_multiarray.data.append(mouse_x)
                    starts_multiarray.data.append(mouse_y)
                    #Start subscribers to motion plans for individual robots
                    rospy.Subscriber("mopat/control/motion_plan_{0}".format(robot_index),
                                      UInt32MultiArray,
                                      robot_list[robot_index].motion_plan_cb)
                    robot_index += 1                #Increment start index
                #If user inputs goal locations
                else:
                    #Goal obstacle overlap check
                    if raw_image[screen_size[1]-mouse_y, mouse_x][0] < 128:
                        #Set robot goal
                        robot_goals[robot_index] = (mouse_x, mouse_y)
                        robot_list[robot_index].set_goal((mouse_x, mouse_y))
                        goals_multiarray.data.append(mouse_x)
                        goals_multiarray.data.append(mouse_y)
                        print("LOG: Got Robot", robot_index,
                              "Goal:", mouse_x,";",mouse_y)
                        robot_index += 1            #Increment goal index
                        if robot_index >= len(robot_list):
                            got_goals = True        #Flip flag to start simulation
                            continue
                    else:
                        print("USER: The point lies within an obstacle")

                got_mouse_click = False
        #Start simulation if all goals found
        else:
            #Start individual robot threads once
            if not started:
                print("LOG: Starting Simulation...")
                for i in robot_list:
                    robot_list[i].start()
                started = True
        #Update screen
        screen.fill((0,0,0))
        space.step(1/steps)
        space.debug_draw(draw_options)
        #After getting starting locations
        if got_starts:
            #Draw goal and update robot_reached_list
            robot_reached_list = []
            for i in range(robot_index):
                robot_list[i].draw_goal(screen)
                pos = robot_list[i].get_pos()
                robot_reached_list.append(robot_list[i].robot_reached)
                #Update robot positions
                positions_multiarray.data.append(int(pos[0]))
                positions_multiarray.data.append(int(pos[1]))
            #Stop if all robots reached
            if sum(robot_reached_list) == robot_index and robot_index != 0:
                print("LOG: All Robots Reached, Simulation Completed!")
                print("EXIT: Exiting simulation in 5s")
                rospy.sleep(5)
                sys.exit(0)
        #Step up
        pygame.display.flip()
        #Publish raw image
        raw_image = conv2matrix(screen, space, draw_options)
        pub_raw.publish(bridge.cv2_to_imgmsg(raw_image, encoding="passthrough"))
        #Publish robot information
        pub_starts.publish(starts_multiarray)
        pub_goals.publish(goals_multiarray)
        pub_positions.publish(positions_multiarray)
        pub_num.publish(len(robot_list))
        #Clear positions for next step
        positions_multiarray.data.clear()
        clock.tick(steps)
        # print(clock.get_fps())

if __name__ == "__main__":
    try:
        simulator_node()
    except rospy.ROSInterruptException:
        pass
