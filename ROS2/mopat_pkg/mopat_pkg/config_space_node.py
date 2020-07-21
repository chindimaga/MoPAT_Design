#MoPAT Design Lab - Guining Pertin
#Configuration space node(ROS2) - 21-05-20

'''
This node generates the configuration space for the given occ_map
Subscribed topics:
    /mopat/tracking/occ_map               -   sensor_msgs/Image (Bool)
Published topics:
    /mopat/tracking/static_config         -   sensor_msgs/Image (Bool)
Work:
    Gets the occupancy map and dilates it to get configuration space
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
#Others
import time
import numpy as np
from scipy import ndimage

class config_space_node(Node):
    def __init__(self, rad):
        #Initialize
        super().__init__("config_space_node")
        self.get_logger().info("INIT: Started configuration space node")
        self.create_subscription(Image, "/mopat/tracking/occ_map", self.occ_map_cb, 2)
        self.pub = self.create_publisher(Image, "/mopat/tracking/config_space", 2)
        #Class variables
        self.bridge = CvBridge()    #CV-ROS bridge
        self.rad = rad
        self.gen_struct()

    def gen_struct(self):
        '''
        Generates circular struct with given rad
        '''
        #Create 4th quadrant of the struct
        struct_4th = np.ones((self.rad, self.rad), dtype=bool)
        #Loop through each point
        for y in range(0, self.rad):
            for x in range(0, self.rad):
                #If distance from 0,0 is greater than rad
                if np.linalg.norm([x,y]) > self.rad:
                    #Set as 0
                    struct_4th[x,y] = False
        #Rotate to form 2nd quadrant
        struct_2nd = np.rot90(struct_4th)
        #Combine to form 2nd section
        struct_comb2 = np.concatenate((struct_2nd, struct_4th), axis=0)
        #Rotate to form 1st section
        struct_comb1 = np.rot90(struct_comb2, 2)
        #Combine to form complete circular struct with given rad
        self.struct = np.concatenate((struct_comb1, struct_comb2), axis=1)

    def get_config_space(self):
        '''
        Perform dilation to get configuration space
        '''
        self.config_space = ndimage.binary_dilation(self.occ_map, structure = self.struct)

    def occ_map_cb(self, msg):
        '''
        Get occupancy map and publish configuration space
        Arguments:
            msg     :   ROS sensor_msgs/Image
        '''
        self.occ_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.get_logger().info("LOG: Got occupancy map")
        #Get configuration space and publish
        self.get_config_space()
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.config_space.astype(np.uint8), encoding="passthrough"))

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = config_space_node(16)
    rclpy.spin(create_node)
    #Close node on exit
    create_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
