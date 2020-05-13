import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

bridge = CvBridge()
def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
    bool_mat = cv_image.astype(bool)
    plt.matshow(bool_mat)
    plt.show()
    print("Got image")

def main():
    rospy.init_node("Config_reader", anonymous = True)
    subs = rospy.Subscriber("mopat/config_space", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
