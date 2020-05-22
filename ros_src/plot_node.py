#MoPAT Design Lab - Guining Pertin
#Plotter node - 19-05-20

'''
This node plots live data from all the nodes
Subscribed topics:
    mopat/motion_plan_{i}        -  std_msgs/UInt32MultiArrays
    mopat/robot_starts           -  std_msgs/UInt32MultiArray
    mopat/robot_goals            -  std_msgs/UInt32MultiArray
    mopat/robot_num              -  std_msgs/UInt32
    mopat/static_config          -  sensor_msgs/Image (Bool)
    mopat/occ_map                -  sensor_msgs/Image (Bool)
Published topics:
    None
Work:
    Uses pyplot to plot motion plans on occ_map+config_space data
'''

#Import libraries
#ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt32MultiArray, UInt32
#Others
import matplotlib.pyplot as plt

#Global variables
occ_map = None
static_config = None
screen_size = None
robot_starts = {}           #Dict - robot_index : robot_start
robot_goals = {}            #Dict - robot_index : robot_goal
robot_num = 0               #Number of robots in sumulation
got_occ_map = False         #Flag - True if occ_map data received
got_static_config = False   #Flag - True if static_config data received
colors = ["red", "blue", "brown", "lawngreen",
          "gold" , "violet","blueviolet", "orange",
          "gainsboro", "springgreen", "deeppink", "cyan"]

bridge = CvBridge()         #Required for rosmsg-cv conversion

def occ_map_cb(data):
    '''
    Get occ_map data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_occ_map
    global occ_map
    global screen_size
    if not got_occ_map:
        occ_map = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        screen_size = occ_map.shape
        got_occ_map = True          #Flip flag
        print("LOG: Got occupancy map")

def config_space_cb(data):
    '''
    Get config_space data
    Arguments:
        data    :   ROS sensor_msgs/Image
    '''
    global got_static_config
    global static_config
    if not got_static_config:
        static_config = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        got_static_config = True    #Flip flag
        print("LOG: Got static configuration space")

def robot_starts_cb(data):
    '''
    Get user defined robot starting locations
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_starts
    for i in range(0,len(data.data)//2):
        robot_starts[i] = (data.data[i*2], data.data[i*2+1])

def robot_goals_cb(data):
    '''
    Get user defined robot goal locations
    Arguments:
        data    :   ROS std_msgs/UInt32MultiArray
    '''
    global robot_goals
    for i in range(0,len(data.data)//2):
        robot_goals[i] = (data.data[i*2], data.data[i*2+1])

def robot_num_cb(data):
    '''
    Get user defined number of robots
    Arguments:
        data    :   ROS std_msgs/UInt32
    '''
    global robot_num
    robot_num = data.data

class robot_plan():
    '''
    The robot plan class
    Parameters:
        index           : robot predefined index
    '''
    def __init__(self, index):
        '''
        Well, initialize
        '''
        self.index = index
        self.got_motion_plan = False    #Set local flag
        #Subscribe to ith robot's motion plan
        rospy.Subscriber("/mopat/motion_plan_{0}".format(index),
                         UInt32MultiArray, self.motion_plan_cb)

    def motion_plan_cb(self, data):
        '''
        Get the motion plan for ith robot
        Arguments:
            data    :   ROS std_msgs/UInt32MultiArray
        '''
        #Clear paths for every reception
        self.gen_pathx = []
        self.gen_pathy = []
        for i in range(0,len(data.data)//2):
            self.gen_pathx.append(data.data[i*2])
            self.gen_pathy.append(data.data[i*2+1])
        self.got_motion_plan = True     #Flip flag
        print("LOG: Got Robot",self.index, "Motion Plan")

def plot_node():
    '''
    Create plotter node
    '''
    #Local variables
    robot_motion_plans = {}                 #Dict - robot_index : robot_plan
    robot_motion_plans_subscribed = False   #Flag - True if all motion plans subscribed
    plotted_occ_config = False              #Flag - True if occupancy+static_config plotted
    #Initialize node
    rospy.init_node("plot_node")
    print("LOG: Started MoPAT Plotter Node")
    #Subscribers
    rospy.Subscriber("/mopat/robot_starts", UInt32MultiArray, robot_starts_cb)
    rospy.Subscriber("/mopat/robot_goals", UInt32MultiArray, robot_goals_cb)
    rospy.Subscriber("/mopat/robot_num", UInt32, robot_num_cb)
    rospy.Subscriber("/mopat/occ_map", Image, occ_map_cb)
    rospy.Subscriber("/mopat/static_config", Image, config_space_cb)
    #Plot stuff
    while not rospy.is_shutdown():
        #Wait for occ_map and static_config
        if not (got_occ_map and got_static_config):
            rospy.sleep(1)
            continue
        #If not plotted yet - plot the main base - once
        if not plotted_occ_config:
            plot_map = occ_map + static_config
            plt.matshow(plot_map)
            plt.title("Occupancy Map + Configuration Space")
            plt.axis("off")
            mngr = plt.get_current_fig_manager()
            mngr.window.wm_geometry("+580+50")  #Sets window location
            plt.show(block = False)
            plotted_occ_config = True           #Flip flag
        #After it has been plotted, plot paths
        else:
            plt.clf()
            plt.matshow(plot_map,fignum = 0)
            plt.title("Occupancy Map + Configuration Space + Plans")
            plt.axis("off")
            #Don't run until simulator publishes robot data
            if robot_num != 0:
                for i in range(robot_num):
                    #Start subscribers for each uninitialized robot
                    if i not in robot_motion_plans.keys():
                        robot_motion_plans[i] = robot_plan(i)
                    #Plot paths if ith robot's motion plan received
                    if robot_motion_plans[i].got_motion_plan:
                        pathx2plot = robot_motion_plans[i].gen_pathx
                        pathy2plot = robot_motion_plans[i].gen_pathy
                        #Don't plot if planner didn't find path
                        if pathx2plot[0] != 99999:
                            plt.plot(pathx2plot,
                                     pathy2plot,
                                     colors[i])
            plt.show(block=False)
            plt.pause(0.01)

if __name__ == "__main__":
    try:
        plot_node()
    except rospy.ROSInterruptException:
        pass
