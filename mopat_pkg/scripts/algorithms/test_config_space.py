import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
import numpy as np

def callback(data):
    print("Found")
rospy.init_node('listener')
rospy.Subscriber("/mopat/config_space", numpy_msg(np.bool), callback)
rospy.spin()
