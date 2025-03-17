#! /usr/bin/env python3


import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CNN_driving_class:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.section = "Road"

    def image_callback(self, msg):
        self.section = "Road"

    def track_section_callback(self, msg):
        self.section = msg.data

def image_subscriber():
    rospy.init_node('driving_code')  # Renamed for clarity

    driver = CNN_driving_class()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, driver.image_callback)
    rospy.Subscriber('/track_section', String, driver.track_section_callback)

    
    # Give ROS some time to set up
    rospy.sleep(1)
    rospy.loginfo("CNN driving node initialized!")
    rospy.spin()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass