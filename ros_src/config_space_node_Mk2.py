#MoPAT Design Lab
#Configuration space node

#Import libraries
#ROS
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#Others
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

#Generator function
def gen_config(map, rad):
    """
    Generates the configuration space for given map and robot
    Arguments:
                map : N x M boolean matrix
                rad : Robot size(assumed circular) in 'map' units
    Output:
                config_space : N x M configuration space
    """
    #Rad sized circular binary structure
    rad_struct = gen_struct(rad)
    #Extend map by rad
    config_space = ndimage.binary_dilation(map, structure = rad_struct)
    #Return
    return config_space

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
    bridge = CvBridge()
    occ_map = np.zeros((100,100), dtype=bool)
    #Rectangle in between
    occ_map[30:80, 30:80] = True
    # config_space.layout.dim[1].size = 100
    #Initialize node
    rospy.init_node("config_space_node", anonymous=True)
    #Subscribe to boolean occupancy map
    # rospy.Subscriber("/mopat/occ_map", ByteMultiArray, occ_map_cb)
    #Publish configuration space
    pub = rospy.Publisher('mopat/config_space', Image, queue_size=5)
    #Set rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Generate configuration space
        temp = gen_config(occ_map, rad)
        #Publish data
        pub.publish(bridge.cv2_to_imgmsg(temp.astype(np.uint8), encoding="passthrough"))
        rate.sleep()

if __name__ == "__main__":
    try:
        config_space_node()
    except rospy.ROSInterruptException:
        pass
