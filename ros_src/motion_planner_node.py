#Guining Pertin - 14-05-20
import sys
import os
import rospy
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from mopat_astar import Astar

bridge = CvBridge()
done = False

def callback(data):
    global done
    if not done:
        conv_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
        bool_mat = conv_image.astype(bool)
        print("Got Config Space")
        plt.matshow(bool_mat)
        plt.show(block=False)
        plt.pause(0.5)
        astar_obj = Astar(0, bool_mat, 0, 0, 100, 100)
        py, px = astar_obj.find_best_route(100 - 0,0, 100 - 99, 99)
        plt.plot(px, py, "red")
        plt.show()
        done = True

def main():
    rospy.init_node("Motion_planner", anonymous = True)
    subs = rospy.Subscriber("mopat/config_space", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()
