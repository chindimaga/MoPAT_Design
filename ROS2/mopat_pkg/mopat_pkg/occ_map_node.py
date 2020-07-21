#MoPAT Design Lab - Guining Pertin
#Occupancy map node(ROS2) - 20-07-20

'''
This node generates the occupancy map given the raw image
Subscribed topics:
    /mopat/testbed/raw_image         -   sensor_msgs/Image (BGR)
Publised topics:
    /mopat/tracking/occ_map           -   sensor_msgs/Image (Bool)
Work:
    Uses cv2 to threshold raw simulator image and converts to boolean obstacle map
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
#Others
import sys
import cv2
import time
import numpy as np

class occ_map_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("occ_map_node")
        self.get_logger().info("INIT: Started occupancy map node")
        self.create_subscription(Image, "/mopat/testbed/raw_image", self.raw_image_cb, 2)
        self.pub = self.create_publisher(Image, "/mopat/tracking/occ_map", 2)
        #Class variables
        self.bridge = CvBridge()    #CV-ROS brige

    def raw_image_cb(self, msg):
        '''
        Get raw_image data and publish the occ_map
        Arguments:
            msg     :   ROS sensor_msgs/Image
        '''
        raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.get_logger().info("LOG: Got raw image")
        #Get occupancy map by segmentation
        gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, occ_map = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((10,10), np.uint8)
        occ_map = cv2.morphologyEx(occ_map, cv2.MORPH_OPEN, kernel)
        #Convert to bool and publish
        self.occ_map = occ_map.astype(bool)  #Necessary conversion
        #Publish only possible in integer format
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.occ_map.astype(np.uint8), encoding = "passthrough"))
        #Wait 5s for next frame
        time.sleep(5)

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = occ_map_node()
    rclpy.spin(create_node)
    #Close node on exit
    create_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
