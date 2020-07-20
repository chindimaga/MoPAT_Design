#MoPAT Design Lab - NAME
#Node name - Start Date

'''
Node definition
Subscribed topics:
    List all topics it subscribes to
Published topics:
    List all topics it publishes
Work:
    The actual working of the node algo in 1 line
'''

#Import libraries
#ROS
import rclpy
from rclpy.node import Node
#Messages
from std_msgs.msg import String
#Others - optional
import sys
import time

#MoPAT algorithm files - optional

#Global variables
msg = String()

#Any other function
def my_func(arg):
    '''
    Definition
    Arguments:
        args    :   xxxxx
    Returns:
        rets    :   xxxxx
    '''

class our_node(Node):

    def __init__(self):
        super().__init__('node_name')
        #Create publishers
        self.pub1 = self.create_publisher(String, 'topic_name', 2)  #(msg_type, topic_name, queue_size)
        #Create subscribers - need not create a variable
        self.create_subscription(String, 'topic_name', self.sub1_cb, 2) #(msg_type, topic_name, callback function, queue_size)

        #YOUR CODE HERE
        msg.data = "Hello Bitches!"
        #Everytime you want to publish use
        pub1.publish(msg)

    #Callback functions
    def sub1_cb(self, msg):
        '''
        Definition
        Arguments:
            msg    :   ROS message type
        '''
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    #Create object
    my_node = our_node()
    #Spin will keep repeating the node until exit
    rclpy.spin(my_node)
    #Close node
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
