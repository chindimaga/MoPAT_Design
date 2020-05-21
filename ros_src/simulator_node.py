#Guining Pertin
#Simulator node - 12-05-20

'''
This node runs the entire simulation(only)
Subscribed topics:
    mopat/motion_plan_{i}        -   std_msgs/UInt32MultiArrays
    mopat/mrc_output_flags       -   std_msgs/ByteMultiArray
Published topics:
    mopat/raw_image         -   sensor_msgs/Image (BGR)
    mopat/robot_starts      -   std_msgs/UInt32MultiArray
    mopat/robot_goals       -   std_msgs/UInt32MultiArray
    mopat/robot_positions   -   std_msgs/UInt32MultiArray
    mopat/robot_num         -   std_msgs/UInt32
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

robot_list = {}

def mrc_cb(data):
    global robot_list
    for i in range(len(data.data)):
        robot_list[i].mrc_flag = data.data[i]

def simulator_node():
    '''
    Function to run the simulation
    '''
    global robot_list
    #Variables
    steps = 50
    bridge = CvBridge()
    robot_starts = {}
    starts_multiarray = UInt32MultiArray()
    robot_goals = {}
    goals_multiarray = UInt32MultiArray()
    positions_multiarray = UInt32MultiArray()
    #Flags
    got_starts = False
    got_goals = False
    started = False
    got_mouse_click = False
    robot_index = 0
    #Game initialization
    os.environ['SDL_VIDEO_WINDOW_POS'] = "+0,+50"
    pygame.init()
    pygame.display.set_caption("MoPAT Multi-Robot Simulator MkII")
    screen = pygame.display.set_mode(screen_size)
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()
    #Create space
    space = pymunk.Space()
    #Create node
    rospy.init_node("simulator_node")
    print("LOG: Started MoPAT Multi-Robot Simulator MkII node")
    #Publishers
    rospy.Subscriber("/mopat/mrc_output_flags", ByteMultiArray, mrc_cb)
    pub_raw = rospy.Publisher("mopat/raw_image", Image, queue_size=5)
    pub_starts = rospy.Publisher("mopat/robot_starts", UInt32MultiArray, queue_size=5)
    pub_goals = rospy.Publisher("mopat/robot_goals", UInt32MultiArray, queue_size=5)
    pub_positions = rospy.Publisher("mopat/robot_positions", UInt32MultiArray, queue_size=5)
    pub_num = rospy.Publisher("mopat/robot_num", UInt32, queue_size=5)
    #Create map
    generate_test_map(space)
    print("USER: Enter initial positions now")
    #Run the simulator
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            #Exiting
            if event.type == QUIT:
                print("LOG: Exiting simulation")
                sys.exit(0)
            elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
                print("LOG: Exiting simulation")
                sys.exit(0)
            #Get user input
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                mouse_y = screen_size[1] - mouse_y
                got_mouse_click = True
            if (event.type == KEYDOWN) and (event.key == K_w) and not got_goals:
                print("USER: Enter goals now")
                robot_index = 0
                goals_multiarray.data.clear()
                got_starts = True   #Take goals
        if not got_goals:
            #If mouse click found
            if got_mouse_click:
                #If not all starts found
                if not got_starts:
                    print("LOG: Got Robot", robot_index,
                          "Start:", mouse_x,";",mouse_y)
                    robot_list[robot_index] = Robot(robot_index, space, (mouse_x, mouse_y))
                    # robot_starts[robot_index] = (mouse_x, mouse_y)
                    starts_multiarray.data.append(mouse_x)
                    starts_multiarray.data.append(mouse_y)
                    #Start subscribers to motion plans for individual robots
                    rospy.Subscriber("/mopat/motion_plan_{0}".format(robot_index),
                                      UInt32MultiArray,
                                      robot_list[robot_index].motion_plan_cb)
                    robot_index += 1
                #Otherwise get goals
                else:
                    #First check if goal overlaps with obstacle
                    if raw_image[screen_size[1]-mouse_y, mouse_x][0] < 128:
                        print("LOG: Got Robot", robot_index,
                              "Goal:", mouse_x,";",mouse_y)
                        robot_goals[robot_index] = (mouse_x, mouse_y)
                        robot_list[robot_index].set_goal((mouse_x, mouse_y))
                        goals_multiarray.data.append(mouse_x)
                        goals_multiarray.data.append(mouse_y)
                        robot_index += 1
                        if robot_index >= len(robot_list):
                            got_goals = True
                            continue
                    else:
                        print("USER: The point lies within an obstacle")

                got_mouse_click = False
        else:
            if not started:
                print("LOG: Starting Simulation...")
                for i in robot_list:
                    robot_list[i].start()
                started = True
            #If simulation started
        #Update screen
        screen.fill((0,0,0))
        space.step(1/steps)
        space.debug_draw(draw_options)
        #After it starts
        if got_starts:
            robot_reached_list = []
            for i in range(robot_index):
                robot_list[i].draw_goal(screen)
                pos = robot_list[i].get_pos()
                robot_reached_list.append(robot_list[i].robot_reached)
                #Get positions
                positions_multiarray.data.append(int(pos[0]))
                positions_multiarray.data.append(int(pos[1]))
                #Also check if the motion plans have been found
            if sum(robot_reached_list) == robot_index and robot_index != 0:
                print("All Robots Reached, Simulation Completed!")
                print("LOG: Exiting simulation in 5s")
                rospy.sleep(5)
                sys.exit(0)
        pygame.display.flip()
        #Get raw iamge
        raw_image = conv2matrix(screen, space, draw_options)
        #Publish raw data
        pub_raw.publish(bridge.cv2_to_imgmsg(raw_image, encoding="passthrough"))
        #Publish robot information
        pub_starts.publish(starts_multiarray)
        pub_goals.publish(goals_multiarray)
        pub_positions.publish(positions_multiarray)
        pub_num.publish(len(robot_list))
        #Clear
        positions_multiarray.data.clear()
        clock.tick(steps)
        # print(clock.get_fps())

if __name__ == "__main__":
    try:
        simulator_node()
    except rospy.ROSInterruptException:
        pass
