#MoPAT Design Lab
#Configuration space generator

#Import libraries
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import numpy as np
from scipy import ndimage

occ_map = np.zeros((100,100), dtype=bool)
#Rectangle in between
occ_map[30:80, 30:80] = True

def gen_config(map, rad):
    """
    Generates the configuration space for given map and robot
    Arguments:
                map : N x M boolean matrix
                rad : Robot size(assumed circular) in 'map' units
    Output:
                config : N x M configuration space
    """
    #Rad sized circular binary structure
    rad_struct = gen_struct(rad)
    #Extend map by rad
    config = ndimage.binary_dilation(map, structure = rad_struct)
    #Return
    return config

def gen_struct(rad):
    """
    Generates circular struct with given rad
    Arguments:
                rad : Robot size(assumed circular) in 'map' units
    Output:
                struct : 2*rad x 2*rad circular struct
    """
    #Create 4th quadrant of the struct
    struct_4th = np.ones((rad, rad), dtype=bool)
    #Loop through each point
    for y in range(0, rad):
        for x in range(0, rad):
            #If distance from 0,0 is greater than rad
            if np.linalg.norm([x,y]) > rad:
                #Set as 0
                struct_4th[x,y] = False
    #Rotate to form 2nd quadrant
    struct_2nd = np.rot90(struct_4th)
    #Combine to form 2nd section
    struct_comb2 = np.concatenate((struct_2nd, struct_4th), axis=0)
    #Rotate to form 1st section
    struct_comb1 = np.rot90(struct_comb2, 2)
    #Combine to form complete circular struct with given rad
    struct = np.concatenate((struct_comb1, struct_comb2), axis=1)
    return struct

def config_space_node():
    """
    Create configuration space node
    """
    rad = 10
    # occ_map = ByteMultiArray()
    config_space = UInt8MultiArray()
    config_space.layout.dim.append(MultiArrayDimension())
    config_space.layout.dim.append(MultiArrayDimension())
    config_space.layout.dim[0].size = 100
    config_space.layout.dim[0].stride = 100*100
    config_space.layout.dim[0].label = "length"
    config_space.layout.dim[1].size = 100
    config_space.layout.dim[1].stride = 100
    config_space.layout.dim[1].label = "breadth"
    config_space.layout.data_offset = 0
    # config_space.layout.dim[1].size = 100
    #Initialize node
    rospy.init_node("config_space_node", anonymous=True)
    #Subscribe to boolean occupancy map
    # rospy.Subscriber("/mopat/occ_map", ByteMultiArray, occ_map_cb)
    #Publish configuration space
    pub = rospy.Publisher('mopat/config_space', UInt8MultiArray, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Generate configuration space
        temp = gen_config(occ_map, rad)
        #Convert to int8 for ROS msg
        temp = temp.astype(np.uint8).tolist()
        config_space.data = temp[0]
        # print(config_space.data)
        # print(type(config_space.data[1]))
        # print(config_space.data)
        #Publish data
        pub.publish(config_space)
        rate.sleep()

# def occ_map_cb(data):
#     """
#     Occupancy map callback function
#     """
#     global occ_map
#     occ_map = data.data

if __name__ == "__main__":
    try:
        config_space_node()
    except rospy.ROSInterruptException:
        pass
