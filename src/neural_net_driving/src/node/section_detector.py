#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import cv2
import numpy as np

lower_pink = np.array([0,  0, 120])    # B=0, G=0, R=120
upper_pink = np.array([100, 100, 255]) # B=100, G=100, R=255


class MapSectionDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.section = 'Road'
        self.pub_sec = rospy.Publisher('/track_section', String, queue_size=1)
        self.last_transition = rospy.Time.now().to_sec()
        self.transition_cooldown = 3
        self.frame_counter = 0
        


    def image_callback(self, msg):
        """
        Receives an Image from ROS, determines what section of the map we are in!
        For detecting what track section we are in please write it in a way agnostic to current infrastructure.
        Right now it is used to label the saved imaged, in future, it will determine the driving model.
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        if rospy.Time.now().to_sec() - self.last_transition > self.transition_cooldown:
            if self.frame_counter >= 5:
                self.frame_counter = 0

                h, w, _ = img.shape

                crop_top = int(0.8 * h)
                bottom_region = img[crop_top:, :]

                lower_pink = np.array([0,  0, 120])    # B=0, G=0, R=120
                upper_pink = np.array([100, 100, 255]) # B=100, G=100, R=255

                # Create a mask of pixels that fall within our "pinkish" BGR range
                pink_mask = cv2.inRange(bottom_region, lower_pink, upper_pink)

                if np.any(pink_mask > 0):
                    self.last_transition = rospy.Time.now().to_sec()
                    if self.section == 'Road':
                        self.section = 'Gravel'
                    if self.section == 'Gravel':
                        self.section = 'OffRoad'
                    if self.section == 'OffRoad':
                        self.section = 'ramp'
                    
            else:
                self.frame_counter+= 1
        
        self.pub_sec.publish("Road")


def image_subscriber():
    rospy.init_node('Section_detector')  # Renamed for clarity

    data_saver = MapSectionDetector()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, data_saver.image_callback)
    
    # Give ROS some time to set up
    rospy.sleep(1)
    rospy.loginfo("section detection node initialized!")
    rospy.spin()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass