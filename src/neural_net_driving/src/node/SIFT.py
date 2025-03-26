#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
import cv2
import numpy as np
from collections import Counter


class sift_class:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_buffer = []
        self.section = None

        self.size_reads = []
        self.size_concensus = None
        self.crime_reads = []
        self.crime_concensus = None
        self.time_reads = []
        self.time_concensus = None
        self.place_reads = []
        self.place_concensus = None
        self.motive_reads = []
        self.motive_concensus = None
        self.weapon_reads = []
        self.weapon_concensus = None
        self.bandit_reads = []
        self.bandit_concensus = None


    def image_callback(self, msg):
        """
        Receives an Image from ROS, converts it using CvBridge, optionally displays it, 
        and conditionally saves it if self.r == True.
        """
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        #the bule of the signs depends on the lighting. it is generally always 100 higher than the red and green channels
        small_img = cv2.resize(cv2_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_LINEAR)
        
            # Split BGR channels
        B, G, R = cv2.split(small_img)

        # Compute boolean masks for your criteria
        red_green_close = np.abs(R.astype(np.int16) - G.astype(np.int16)) <= 10
        blue_dominant = (((B.astype(np.int16) - R.astype(np.int16)) >= 90) & ((B.astype(np.int16) - R.astype(np.int16)) <= 110)) & \
                        (((B.astype(np.int16) - G.astype(np.int16)) >= 90) & ((B.astype(np.int16) - G.astype(np.int16)) <= 110))

        # Combine masks
        blue_mask = red_green_close & blue_dominant

        # Create the output binary image (white for matching, black otherwise)
        result = np.zeros_like(B, dtype=np.uint8)
        result[blue_mask] = 255
        white_pixels = np.count_nonzero(result == 255)
        binary_bgr = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)

        percentage_blue = white_pixels/result.size*100

        text = f"{percentage_blue:.2f}%"
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (10, 30)  # Bottom-left corner of text
        fontScale = 1
        color = (0, 0, 255)
        thickness = 2

        # Draw text on the image
        cv2.putText(binary_bgr, text, org, font, fontScale, color, thickness, cv2.LINE_AA)

        cv2.imshow("feed", binary_bgr)
        cv2.waitKey(1)
        
        #if 0.55% of the image falls into the blue range, there is most likley a 
        if percentage_blue >= 0.55:

            B, G, R = cv2.split(cv2_img)
            # Compute boolean masks for your criteria
            red_green_close = np.abs(R.astype(np.int16) - G.astype(np.int16)) <= 10
            blue_dominant = (((B.astype(np.int16) - R.astype(np.int16)) >= 90) & ((B.astype(np.int16) - R.astype(np.int16)) <= 110)) & \
                            (((B.astype(np.int16) - G.astype(np.int16)) >= 90) & ((B.astype(np.int16) - G.astype(np.int16)) <= 110))

            # Combine masks
            blue_mask = red_green_close & blue_dominant

            # Create the output binary image (white for matching, black otherwise)
            result = np.zeros_like(B, dtype=np.uint8)
            result[blue_mask] = 255
            binary_bgr = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)

            #self.image_buffer.append(binary_bgr)

    # TODO: implement this function. it has to write to one of the sign-read 
    # lists use  a cnn to clasify the sign type and use another cnn to read the message
    #images are already black and white as per the mask.
    def process_a_image(self):
        if len(self.image_buffer)>0:
            image = self.image_buffer.pop()
            
            sign_title = "yup"
            message = "what the message is"

            if sign_title == "size":
                self.size_reads.append(message)
            elif sign_title == "crime":
                self.crime_reads.append(message)
            elif sign_title == "time":
                self.time_reads.append(message)
            elif sign_title == "place":
                self.place_reads.append(message)
            elif sign_title == "motive":
                self.motive_reads.append(message)
            elif sign_title == "weapon":
                self.weapon_reads.append(message)
            elif sign_title == "bandit":
                self.bandit_reads.append(message)

        return

    def track_section_callback(self, msg):
        self.section = msg.data

    def majority_vote(strings):
        if not strings:
            return ""

        max_len = max(len(s) for s in strings)
        padded = [s.ljust(max_len) for s in strings]  # pad with spaces

        result = ""
        for i in range(max_len):
            chars = [s[i] for s in padded]
            most_common = Counter(chars).most_common(1)[0][0]
            result += most_common

        return result.strip()  # strip padding


def image_subscriber():
    rospy.init_node('SIFT_node')  # Renamed for clarity

    sift = sift_class()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, sift.image_callback)
    rospy.Subscriber('/track_section', String, sift.track_section_callback)

    # Give ROS some time to set up
    rospy.sleep(1)

    rospy.loginfo("SIFT node initialized!")

    rate = rospy.Rate(5)  # 5Hz 
    while not rospy.is_shutdown():
        sift.process_a_image()
        rate.sleep()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
