#MoPAT Design Lab - Rahul D, Guining Pertin
#Stream server node(ROS2) - 08-09-20

'''
This node gets the raw image topic, hosts it on the http server
and also get the use inputs
Published topics:
    /mopat/robot/robot_goals        -
Subscribed topics:
    /mopat/testbed/raw_image       -
    /mopat/tracking/robot_location  -
    /mopat/control/motion_plan_{i}  -
Work:
    Runs a flask app for the http server
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
#ROS messages
from sensor_msgs.msg import Image
#Others
from algorithms.videocap import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
import threading
import argparse
import datetime
import imutils
import time
import cv2

#Parameters
ip = "0.0.0.0"
port = "8000"

#Global variables
outputFrame = None
create_node = None
lock = threading.Lock()

# initialize a flask object
app = Flask(__name__)

class stream_node(Node):
    def __init__(self):
        #Initialize
        super().__init__("stream_node")
        self.get_logger().info("INIT")
        self.create_subscription(Image, "/mopat/testbed/raw_image", self.raw_image_cb, 2)
        self.bridge = CvBridge()

    def raw_image_cb(self, msg):
        '''
        Get raw image
        '''
        global outputFrame, lock
        #Get data
        self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # self.get_logger().info("LOG: Got raw image")
        # acquire the lock, set the output frame, and release the
        # lock
        with lock:
        	outputFrame = self.raw_image.copy()

def generate():
    global outputFrame, lock
    # loop over frames from the output stream
    while True:

        with lock:
            if outputFrame is None:
                continue
            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
            # ensure the frame was successfully encoded
            if not flag:
                continue

        # yield the output frame in the byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
        bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index():
	# return the rendered template
	return render_template("index.html")

@app.route("/video_feed")
def video_feed():
	# response generated and type
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

def keep_it_spinning():
    global create_node
    rclpy.spin(create_node)

def keep_flask_running():
    global app
    # start the flask app
    app.run(host=ip, port=port, debug=True,threaded=True, use_reloader=False)

def main():
    global create_node
    #ROS node
    rclpy.init()
    create_node = stream_node()
    # flask_thread = threading.Thread(target=keep_flask_running)
    # flask_thread.daemon = True
    spin_thread = threading.Thread(target=keep_it_spinning)
    spin_thread.daemon = True
    try:
        # flask_thread.start()
        # rclpy.spin(create_node)
        spin_thread.start()
        app.run(host=ip, port=port, debug=True,threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        create_node.get_logger().info("EXIT")
        #Close node on exit
        create_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
