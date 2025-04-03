#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
import cv2
import datetime
from geometry_msgs.msg import Twist
import os

IMG_HEIGHT = 316
IMG_WIDTH = 384 

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
            
            save_image = cv2.resize(cv2_img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_LINEAR)
            cv2_img = cv2.resize(cv2_img, (IMG_WIDTH*2, IMG_HEIGHT*2), interpolation=cv2.INTER_LINEAR)
            # If 'r' (record) is True, save the image
            if self.r:
                # Make a filename from current time + joystick values
                now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"image_{now_str}_{self.track_section}_Lin_{self.lin:.2f}_Ang_{self.ang:.2f}.png"

                filename ='/home/fizzer/ros_ws/training_for_driving/'+filename
                
                cv2.imwrite(filename, save_image)

            text = self.track_section
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (10, 30)  # Bottom-left corner of text
            fontScale = 1
            color = (0, 0, 255) if self.r else (0, 255, 0)  # Green if recording, red otherwise
            thickness = 2

            # Draw text on the image
            cv2.putText(cv2_img, text, org, font, fontScale, color, thickness, cv2.LINE_AA)

            # Show the image in an OpenCV window (non-blocking)
            cv2.imshow("feed", cv2_img)
            cv2.waitKey(1)

    def cmd_vel_callback(self, msg):
        self.lin = msg.linear.x
        self.ang = msg.angular.z

    def record_callback(self, msg):
        self.r = msg.data

    def track_section_callback(self, msg):
        self.track_section = msg.data

def image_subscriber():
    rospy.init_node('image_writer')  # Renamed for clarity

    data_saver = writing_Images()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, data_saver.image_callback)
    rospy.Subscriber('/B1/cmd_vel', Twist, data_saver.cmd_vel_callback)
    rospy.Subscriber('/record_topic', Bool, data_saver.record_callback)
    rospy.Subscriber('/track_section', String, data_saver.track_section_callback)

    # Give ROS some time to set up
    rospy.sleep(1)

    rospy.loginfo("Image capturing node initialized!")
    rospy.spin()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
