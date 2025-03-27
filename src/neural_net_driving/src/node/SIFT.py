#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
import cv2
import numpy as np
from collections import Counter

# ------------------------------------------------------------------------------
#                               CONFIGURATION
# ------------------------------------------------------------------------------
BLUE_SIGN_THRESHOLD = 0.55  # If >= this fraction of image is "blue," we assume a sign is present

# ------------------------------------------------------------------------------
#                           CLASS: sift_class
# ------------------------------------------------------------------------------
class sift_class:
    """
    This class is under development to detect text on blue signs while the robot
    drives around the course. Signs are identified by a distinct 'blue' region.
    """

    def __init__(self):
        self.bridge = CvBridge()

        # For storing sign images that might be recognized
        self.image_buffer = []
        self.section = None

        # Various placeholders for recognized sign content
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

        # Publish status
        self.pub_status = rospy.Publisher('/SIFT_node/status', String, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def publish_status(self, event):
        """
        Periodically publish how many images are in buffer, plus the current section.
        """
        status_str = (
            f"section={self.section}, "
            f"image_buffer_len={len(self.image_buffer)}"
        )
        self.pub_status.publish(String(data=status_str))

    def image_callback(self, msg):
        """
        Receives an Image from ROS, checks for a large 'blue' region (indicating a sign).
        """
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        # Downscale for color check
        small_img = cv2.resize(cv2_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_LINEAR)
        B, G, R = cv2.split(small_img)

        red_green_close = np.abs(R.astype(np.int16) - G.astype(np.int16)) <= 10
        blue_dominant = (
            ((B.astype(np.int16) - R.astype(np.int16)) >= 90) & ((B.astype(np.int16) - R.astype(np.int16)) <= 110) &
            ((B.astype(np.int16) - G.astype(np.int16)) >= 90) & ((B.astype(np.int16) - G.astype(np.int16)) <= 110)
        )

        blue_mask = red_green_close & blue_dominant

        result = np.zeros_like(B, dtype=np.uint8)
        result[blue_mask] = 255

        white_pixels = np.count_nonzero(result == 255)
        percentage_blue = (white_pixels / result.size) * 100

        # Debug window
        binary_bgr = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
        cv2.putText(
            binary_bgr,
            f"{percentage_blue:.2f}%",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )
        # cv2.imshow("feed", binary_bgr)
        # cv2.waitKey(1)
        
        # If enough of the image is 'blue,' assume a sign is present
        if (percentage_blue / 100.0) >= BLUE_SIGN_THRESHOLD:
            # Potentially store a higher-res mask in self.image_buffer
            # (commented out for now)
            pass

    def process_a_image(self):
        """
        Takes one buffered image (if any), identifies the sign's category
        and text, and stores the result in the corresponding list.
        """
        if len(self.image_buffer) > 0:
            image = self.image_buffer.pop()
            
            # Placeholder classification
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

    def track_section_callback(self, msg):
        """
        Saves the current map section.
        """
        self.section = msg.data

    def majority_vote(strings):
        """
        Example function to merge multiple noisy sign readings
        by character-level majority voting.
        """
        if not strings:
            return ""
        max_len = max(len(s) for s in strings)
        padded = [s.ljust(max_len) for s in strings]
        result = ""
        for i in range(max_len):
            chars = [s[i] for s in padded]
            most_common = Counter(chars).most_common(1)[0][0]
            result += most_common
        return result.strip()

# ------------------------------------------------------------------------------
#                              MAIN ROS NODE
# ------------------------------------------------------------------------------
def image_subscriber():
    rospy.init_node('SIFT_node')  # Renamed for clarity

    sift = sift_class()

    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, sift.image_callback)
    rospy.Subscriber('/track_section', String, sift.track_section_callback)

    rospy.sleep(1)
    rospy.loginfo("SIFT node initialized!")

    rate = rospy.Rate(5)  # 5Hz
    while not rospy.is_shutdown():
        # Periodically process one buffered image (if any)
        sift.process_a_image()
        rate.sleep()

if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
