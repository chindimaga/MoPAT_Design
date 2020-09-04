#MoPAT Design Lab - Guining Pertin
#Camera output node(ROS2) - 04-09-20

'''
This node gets the raw image and localizes the robots
Subscribed topics:
    /mopat/testbed/raw_image            -   sensor_msgs/Image (BGR)
Published topics:
    /mopat/tracking/loc_data            -   std_msgs/UInt32MultiArrays
Work:
    Uses blob detectors to track robots
'''

#ROS
import rclpy
from rclpy.node import Node
from cv_bridge  import CvBridge
#ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, MultiArrayLayout
#Others
import os
import numpy as np
import cv2
import time
#MoPAT
from mopat_pkg.detect_blob import detect_blob
from mopat_pkg.import_mask_params import import_mask_params
from mopat_pkg.localize import localize

class localization_node(Node):
    def __init__(self):
        super().__init__("localization_node")
        self.get_logger().info("INIT")
        self.create_subscription(Image, "/mopat/testbed/raw_image", self.raw_image_cb, 2)
        self.pub = self.create_publisher(UInt32MultiArray, "/mopat/tracking/loc_data", 2)
        self.bridge = CvBridge()
        #This is current directory
        dir = os.path.dirname(os.path.realpath(__file__))
        #Get mask params
        self.mask_params = import_mask_params(dir+"/data/mask_parameters")

    def raw_image_cb(self, msg):
        '''
        Get raw_image data and publish loc_data
        Arguments:
        msg     :   ROS sensor_msgs/Image
        '''
        self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.get_logger().info("LOG: Got raw image")
        #Now detect blobs from the given image
        LED_list = detect_blob(self.raw_image, self.mask_params)
        robot_list = localize(LED_list)
        #Publish this list
        self.pub.publish(self.conv2multiarray(robot_list))
        time.sleep(5)

    def conv2multiarray(self, list):
        '''
        Convert the robot list to a Multiarray
        Arguments:
        list    :   robot_list
        '''
        robot_list = UInt32MultiArray()
        #Flatten out list as the data for multiarray
        for robot in list:
            for loc in robot:
                robot_list.data.append(int(loc))
        return robot_list

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = localization_node()
    try:
        rclpy.spin(create_node)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
