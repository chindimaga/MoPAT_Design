#Guining Pertin
#Multi-robot coordinator node - 16-05-20

'''
This node coordinates the control of all the robots
Subscribed_topics:
    mopat/robot_positions       -   std_msgs/UInt32MultiArray
    mopat/motion_plans_done     -   std_msgs/Bool
Published_topics:
    mopat/mrc_output_flags             -   std_msgs/ByteMultiArray
Work:
    Coordinate start of all robots together
    Coordinate collision control of robots
    Coordinate task distribution for HL tasks
'''
#Import libraries
#ROS
import rospy
from std_msgs.msg import UInt32MultiArray, Bool, ByteMultiArray
#Other
import numpy as np
robot_positions = {}
mrc_local_flags = {}
mrc_output_flags = ByteMultiArray()
motion_plans_done = False

'''
Run     :   0b00000000
Wait    :   0b00000001
Speedx2 :   0b0000001x
'''

def robot_positions_cb(data):
    '''
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_positions
    for i in range(0,len(data.data)//2):
        robot_positions[i] = (data.data[i*2], data.data[i*2+1])

def motion_plans_done_cb(data):
    global motion_plans_done
    motion_plans_done = data.data
    # print(data.data)

def multi_robot_coordinator_node():
    '''
    Create Multi-Robot Coordinator Node
    '''
    #Initialize node
    rospy.init_node("multi_robot_coordinator_node")
    print("LOG: Started Multi-Robot Coordinator Node")
    #Set subscribers
    rospy.Subscriber("/mopat/robot_positions", UInt32MultiArray, robot_positions_cb)
    rospy.Subscriber("/mopat/motion_plans_done", Bool, motion_plans_done_cb)
    #Set publisher
    pub = rospy.Publisher("/mopat/mrc_output_flags", ByteMultiArray, queue_size = 5)
    # rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        robot_num = len(robot_positions)
        #To know that simulation started
        if robot_num != 0:
            #If all motion_plans not generated - wait signal
            #Clear everytime
            mrc_output_flags.data.clear()
            if not motion_plans_done:
                for i in range(robot_num):
                    mrc_output_flags.data.append(0b00000001) #Wait for each robot
            #If all plans done
            else:
                for i in range(robot_num):
                    mrc_output_flags.data.append(0b00000000) #Run each robot
            print(mrc_output_flags.data)
            pub.publish(mrc_output_flags)
        rate.sleep()

if __name__ == "__main__":
    try:
        multi_robot_coordinator_node()
    except rospy.ROSInterruptException:
        pass
