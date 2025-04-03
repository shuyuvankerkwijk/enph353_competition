#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool
import cv2
import numpy as np

# ------------------------------------------------------------------------------
#                                CONFIGURATION
# ------------------------------------------------------------------------------
LOWER_PINK_BOUNDS = np.array([240, 0, 240])  # Pink detection lower bound
UPPER_PINK_BOUNDS = np.array([255, 10, 255]) # Pink detection upper bound
TRANSITION_COOLDOWN_SEC = 5

# ------------------------------------------------------------------------------
#                     CLASS: MapSectionDetector
# ------------------------------------------------------------------------------
class MapSectionDetector:
    """
    Uses camera images to detect when the robot crosses a pink line on the ground,
    transitioning to a new section (Road -> Gravel -> OffRoad -> ramp).
    Publishes the current track section on /track_section.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.pub_sec = rospy.Publisher('/track_section', String, queue_size=1)
        self.pub_status = rospy.Publisher('/section_detector/status', String, queue_size=1)

        # Default to 'Road'
        self.section = 'Road'

        # For rate-limiting section transitions
        self.last_transition_time = rospy.Time.now().to_sec()
        self.detection_active = True

        # Publish status once a second
        rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def publish_status(self, event):
        """
        Publishes the current detection-active state and which section we're in.
        """
        status_msg = String()
        status_msg.data = f"section={self.section}, detection_active={self.detection_active}"
        self.pub_status.publish(status_msg)

    def image_callback(self, msg):
        """
        Receives camera images, checks the bottom region for pink color,
        and if found (beyond a threshold) transitions to the next section
        if the pink line was gone for at least TRANSITION_COOLDOWN_SEC.
        """
        if not self.detection_active:
            return

        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        # Focus on the bottom ~20% of the image
        height, width, _ = img_bgr.shape
        crop_top_index = int(0.8 * height)
        bottom_region = img_bgr[crop_top_index:, :]

        # Pink mask
        pink_mask = cv2.inRange(bottom_region, LOWER_PINK_BOUNDS, UPPER_PINK_BOUNDS)
        pink_visible = np.mean(pink_mask) > 0.2

        current_time = rospy.Time.now().to_sec()

        if pink_visible:
            if current_time - self.last_transition_time >= TRANSITION_COOLDOWN_SEC:
                if self.section == 'Road':
                    self.section = 'Gravel'
                elif self.section == 'Gravel':
                    self.section = 'OffRoad'
                elif self.section == 'OffRoad':
                    self.section = 'ramp'
                self.last_transition_time = current_time  # prevent multiple transitions
            # If pink is still in view, reset the cooldown timer
            self.last_transition_time = current_time

        self.pub_sec.publish(self.section)

    def reset_callback(self, msg):
        data = msg.data
        if data == 'Void':
            return

        self.last_transition_time = rospy.Time.now().to_sec()
        if data == 'Reset':
            self.section = 'Road'
        elif data == 'Gravel':
            self.section = 'Gravel'
        elif data == 'OffRoad':
            self.section = 'OffRoad'
        elif data == 'ramp':
            self.section = 'ramp'

    def teleop_state_callback(self, msg):
        teleop_enabled = msg.data
        # detection_active stays true if either teleop or auto were ever true
        self.detection_active = teleop_enabled or self.detection_active

    def auto_state_callback(self, msg):
        auto_enabled = msg.data
        self.detection_active = auto_enabled or self.detection_active

# ------------------------------------------------------------------------------
#                              MAIN ROS NODE
# ------------------------------------------------------------------------------
def section_detection_node():
    rospy.init_node('Section_detector')

    map_section_detector = MapSectionDetector()

    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, map_section_detector.image_callback)
    rospy.Subscriber('/reset', String, map_section_detector.reset_callback)
    rospy.Subscriber('/teleop', Bool, map_section_detector.teleop_state_callback)
    rospy.Subscriber('/auto', Bool, map_section_detector.auto_state_callback)

    rospy.sleep(1)
    rospy.loginfo("section detection node initialized!")
    rospy.spin()

if __name__ == "__main__":
    try:
        section_detection_node()
    except rospy.ROSInterruptException:
        pass
