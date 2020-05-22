#MoPAT Design Lab - Guining Pertin
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

'''
Commands:
    Run     :   0b00000000
    Wait    :   0b00000001
    Speedx2 :   0b0000001x
'''


#Import libraries
#ROS
import rospy
from std_msgs.msg import UInt32MultiArray, Bool, ByteMultiArray

#Global variables
robot_positions = {}                    #Dict - robot_index : robot_position
mrc_local_flags = {}                    #Dict - robot_index : mrc byte flag
motion_plans_done = False               #Flag - True if all motion plans flag received

def robot_positions_cb(data):
    '''
    Get current robot positions
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_positions
    for i in range(0,len(data.data)//2):
        robot_positions[i] = (data.data[i*2], data.data[i*2+1])

def motion_plans_done_cb(data):
    '''
    Get motion_plans_done flag
    Arguments:
        data    :   ROS std_msgs/Bool
    '''
    global motion_plans_done
    motion_plans_done = data.data      #Update flag

def multi_robot_coordinator_node():
    '''
    Create Multi-Robot Coordinator Node
    '''
    #Local variables
    mrc_output_flags = ByteMultiArray()     #Byte Flag - ByteMultiArray type rosmsg
    #Initialize node
    rospy.init_node("multi_robot_coordinator_node")
    print("LOG: Started Multi-Robot Coordinator Node")
    #Subscribers and publisher
    rospy.Subscriber("/mopat/robot_positions", UInt32MultiArray, robot_positions_cb)
    rospy.Subscriber("/mopat/motion_plans_done", Bool, motion_plans_done_cb)
    pub = rospy.Publisher("/mopat/mrc_output_flags", ByteMultiArray, queue_size = 5)
    #Set rate
    rate = rospy.Rate(1)
    #Coordinate!
    while not rospy.is_shutdown():
        robot_num = len(robot_positions)
        #Don't start until simulation started
        if robot_num != 0:
            #1. If all motion_plans not generated - wait signal
            mrc_output_flags.data.clear()       #Clear flags everytime
            if not motion_plans_done:
                for i in range(robot_num):
                    mrc_output_flags.data.append(0b00000001) #Wait for each robot
            else:
                for i in range(robot_num):
                    mrc_output_flags.data.append(0b00000000) #Run each robot
            pub.publish(mrc_output_flags)
        rate.sleep()

if __name__ == "__main__":
    try:
        multi_robot_coordinator_node()
    except rospy.ROSInterruptException:
        pass
