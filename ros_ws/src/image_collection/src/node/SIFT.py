#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class writing_Images:

    def __init__(self):
        self.lin = 0
        self.ang = 0
        self.r = False
        self.bridge = CvBridge()
        self.track_section = "None"


    def image_callback(self, msg):
            """
            Receives an Image from ROS, converts it using CvBridge, optionally displays it, 
            and conditionally saves it if self.r == True.
            """
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                return
            
            

def image_subscriber():
    rospy.init_node('SIFT_node')  # Renamed for clarity

    data_saver = writing_Images()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, data_saver.image_callback)

    # Give ROS some time to set up
    rospy.sleep(1)

    rospy.loginfo("SIFT node initialized!")
    rospy.spin()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
