#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def hello_world_publisher():
    """
    A simple ROS node that publishes "hello world" messages.
    """
    # Initialize the ROS node
    rospy.init_node('hello_world_publisher', anonymous=True)
    
    # Create a publisher for String messages on the 'hello_world' topic
    pub = rospy.Publisher('hello_world', String, queue_size=10)
    
    # Set the publishing rate (1 Hz)
    rate = rospy.Rate(1)
    
    rospy.loginfo("Hello World Publisher node started. Publishing to /hello_world topic.")
    
    # Keep publishing until the node is shut down
    while not rospy.is_shutdown():
        # Create the message
        hello_str = "Hello World! Time: %s" % rospy.get_time()
        
        # Publish the message
        pub.publish(hello_str)
        
        # Log the published message
        rospy.loginfo("Published: %s", hello_str)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_world_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hello World Publisher node stopped.")
