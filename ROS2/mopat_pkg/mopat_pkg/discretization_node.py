#MoPAT Design Lab - Guining Pertin
#Discretization node(ROS2) - 21-07-20

'''
This node discretizes the configuration space into 1/2^2 scale
for faster path planning
Subscribed topics:
    /mopat/control/static_config        -   sensor_msgs/Image (Bool)
Published topics:
    /mopat/control/discrete_config       -   sensor_msgs/Image (Bool)
Work:
    Samples using numpy to get "blocky" config space
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
#Others
import numpy as np

class discretization_node(Node):
    def __init__(self, samp_size):
        #Initialize
        super().__init__("discretization_node")
        self.get_logger().info("INIT: Started discretization node")
        self.create_subscription(Image, "/mopat/control/static_config", self.static_config_cb, 2)
        self.pub = self.create_publisher(Image, "/mopat/control/discrete_config", 2)
        #Class variables
        self.bridge = CvBridge()    #CV-ROS bridge
        self.samp_size = samp_size
        self.end_sim = False
        self.declare_parameter("samp_size")
        #Get sampling size param
        self.samp_size = self.get_parameter("samp_size").get_parameter_value().integer_value
        self.get_logger().info("LOG: Using samp_size={0} parameter value".format(self.samp_size))

    def static_config_cb(self, msg):
        '''
        Get config space and publish discretized space
        Arguments:
            msg     :   ROS sensor_msgs/Image
        '''
        self.static_config = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        self.get_logger().info("LOG: Got static configuration space")
        #Get configuration space and publish
        self.discrete_config = self.static_config[::self.samp_size, ::self.samp_size]
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.discrete_config.astype(np.uint8), encoding="passthrough"))

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = discretization_node(2)
    try:
        rclpy.spin(create_node)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT: Closing discretization node")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
