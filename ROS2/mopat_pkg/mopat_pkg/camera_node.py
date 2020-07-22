#MoPAT Design Lab - Guining Pertin
#Camera output node(ROS2) - 20-07-20

'''
This node gets the raw image from the camera and publishes it
Published topics:
    /mopat/test_bed/raw_image           -   sensor_msgs/Image (BGR)
Work:
    Uses OpenCV to turn on the camera and get raw image
'''

#ROS
import rclpy
from rclpy.node import Node
from cv_bridge  import CvBridge
#ROS messages
from sensor_msgs.msg import Image
#Others
import os
import numpy as np
import cv2

class camera_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("camera_node")
        self.get_logger().info("INIT")
        self.pub = self.create_publisher(Image, "/mopat/testbed/raw_image", 2)
        #Class variables
        self.bridge = CvBridge()    #CV-ROS bridge

    def run(self, cam_index=0):
        #Start camera
        cap = cv2.VideoCapture(cam_index)
        while 1:
            #Read frame and publish
            _, frame = cap.read()
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame.astype(np.uint8), encoding="passthrough"))
            cv2.imshow("/mopat/testbed/raw_image", frame)
            if cv2.waitKey(1) == ord("q"): break
        #Close cam
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init()
    #Create and run
    create_node = camera_node()
    try:
        create_node.run(1)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
