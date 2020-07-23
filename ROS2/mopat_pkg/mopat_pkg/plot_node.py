#MoPAT Design Lab - Guining Pertin
#Plotter node(ROS2) - 22-07-20

'''
This node plots live data from all the nodes
Subscribed topics:
    /mopat/control/motion_plan_{i}          -  std_msgs/UInt32MultiArrays
    /mopat/robot/robot_starts               -  std_msgs/UInt32MultiArray
    /mopat/robot/robot_goals                -  std_msgs/UInt32MultiArray
    /mopat/tracking/static_config           -  sensor_msgs/Image (Bool)
    /mopat/tracking/occ_map                 -  sensor_msgs/Image (Bool)
Published topics:
    None
Work:
    Uses pyplot to plot motion plans on occ_map+config_space data
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, UInt32
#Others
import matplotlib.pyplot as plt
import numpy as np

class plot_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("plot_node")
        self.get_logger().info("INIT")
        self.create_subscription(Image, "/mopat/tracking/occ_map", self.occ_map_cb, 2)
        self.create_subscription(Image, "/mopat/control/static_config", self.static_config_cb, 2)
        #Class variables
        self.bridge = CvBridge()    #CV-ROS bridge
        self.got_occ_map = False
        self.fig = plt.figure()
        self.fig.add_axes([0,0,1,1])
        self.fig.axes[0].set_title("Occupancy Map + Configuration Space")
        self.fig.axes[0].get_xaxis().set_visible(False)
        self.fig.axes[0].get_yaxis().set_visible(False)

    def occ_map_cb(self, msg):
        '''
        Get occupancy map
        '''
        self.occ_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.got_occ_map = True
        self.get_logger().info("LOG: Got occupancy map")

    def static_config_cb(self, msg):
        '''
        Get configuration space
        '''
        self.config_space = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info("LOG: Got static configuration space")
        #Plot if occ_map is also found
        if self.got_occ_map:
            self.run()

    def run(self):
        '''
        Plot combined occ_map and config_space
        '''
        self.plot_map = self.occ_map + self.config_space
        self.fig.axes[0].matshow(self.plot_map)
        self.fig.show()

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = plot_node()
    try:
        rclpy.spin(create_node)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
