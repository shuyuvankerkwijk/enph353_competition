#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tensorflow as tf
import numpy as np
from neural_net_driving.srv import ImageProcessor, ImageProcessorResponse
import re
from collections import deque

# ------------------------------------------------------------------------------
#                                CONFIGURATION
# ------------------------------------------------------------------------------
IMG_HEIGHT = 316                  # Height for resizing input image to the CNN
IMG_WIDTH = 384                   # Width for resizing input image to the CNN
NN_OUTPUT_SCALAR = 1.0          # Output scaling factor for predicted velocities
CROSSWALK_RED_RATIO_THRESHOLD = 0.05  # % of red pixels to detect crosswalk
MOTION_DETECTION_THRESHOLD = 15       # Threshold for pixel differences
MOTION_COUNT_FOR_ACTIVE_CROSSING = 500                 # If motion is <= this, we assume pedestrian is gone
CROSSWALK_SPEED_UP = 1.5
CROSSWALK_CROSSING_TIME = 2

STATUS_PUBLISH_RATE_HZ = 2
SERVICE_REGEX = r'(.*?) -> lin:([-\d\.]+), ang:([-\d\.]+)'
VALID_SECTIONS = ["Road", "Gravel", "OffRoad", "ramp"]

# ------------------------------------------------------------------------------
#                           CLASS: CNNDrivingNode
# ------------------------------------------------------------------------------
class DrivingNode:
    """
    Subscribes to camera images and uses multiple CNN models for different
    track sections to predict forward and angular velocity commands.

    Also handles logic for stopping at a red crosswalk line and waiting
    until the pedestrian has crossed.
    """

    def __init__(self):
        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_status = rospy.Publisher('/driving_code/status', String, queue_size=1)

        # Section name to decide which model to use
        self.section = None

        # CV Bridge for image conversions
        self.bridge = CvBridge()

        # Will store the current model based on self.section
        self.model = None

        # Whether auto-driving is enabled
        self.auto_enabled = False

        # Crosswalk logic
        self.has_not_reached_crosswalk = True    # True until first red line detection
        self.waiting_for_pedestrian = False      # True once red line is detected
        self.previous_pedestrian_image = None    # For motion detection across frames
        self.time_starting_crosswalk = None
        self.pedestrian_crossing = False
        self.pedestian_queue = deque(maxlen=10)
        for i in range(10):
            self.pedestian_queue.append(False)

    def image_callback(self, msg):
        """
        Callback for camera images. Resizes and normalizes them before passing
        into the currently selected CNN model. Publishes the resulting velocity
        if auto is enabled.
        
        Also implements crosswalk detection:
         - If we see enough red in the lower portion of the image, stop and wait.
         - Then we look for movement in the mid portion of the frame to ensure
           the pedestrian is gone before proceeding.
        """
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
    
        if self.auto_enabled and self.section in VALID_SECTIONS:
            cv2_resized = cv2.resize(cv2_img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_LINEAR)

            cv2.imshow("feed", cv2_img)
            cv2.waitKey(1)

            # Check for crosswalk if we haven't stopped yet
            if self.has_not_reached_crosswalk:
                height = cv2_resized.shape[0]
                bottom_section = cv2_resized[int(height * 4/5):, :, :]

                B_chan, G_chan, R_chan = cv2.split(bottom_section)
                tolerance = 5
                red_mask = (R_chan >= (255 - tolerance)) & (G_chan <= tolerance) & (B_chan <= tolerance)
                red_ratio = np.count_nonzero(red_mask) / red_mask.size

                if red_ratio >= CROSSWALK_RED_RATIO_THRESHOLD:
                    self.waiting_for_pedestrian = True
                    self.has_not_reached_crosswalk = False
                    velocity = Twist()
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.0
                    self.pub_cmd_vel.publish(velocity)

            # If already at crosswalk, wait until pedestrian is gone
            if self.waiting_for_pedestrian:
                height = cv2_img.shape[0]
                mid_section = cv2_img[int(height * 3/8):int(height * 1/2), :, :]

                if self.previous_pedestrian_image is not None:

                    velocity = Twist()
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.0
                    self.pub_cmd_vel.publish(velocity)
                
                    b_old, g_old, r_old = cv2.split(self.previous_pedestrian_image)
                    b_new, g_new, r_new = cv2.split(mid_section)
                    diff = 0.4 * cv2.absdiff(b_old, b_new) + \
                            0.1 * cv2.absdiff(g_old, g_new) + \
                            0.5 * cv2.absdiff(r_old, r_new)
                    self.previous_pedestrian_image = mid_section
                    _, motion_mask = cv2.threshold(diff, MOTION_DETECTION_THRESHOLD, 255, cv2.THRESH_BINARY)
                    motion_amount = np.sum(motion_mask) / 255
                    rospy.loginfo(f"motion amount: {motion_amount}")
                    cv2.imshow("diff", motion_mask)

                    if motion_amount >= MOTION_COUNT_FOR_ACTIVE_CROSSING:
                        self.pedestian_queue.append(True)
                    else:
                        self.pedestian_queue.append(False)

                    if all(self.pedestian_queue): #this check for if he is activley crossing the crosswalk
                        self.pedestrian_crossing = True
                        return

                    #this checks for if we watched him activley cross the crosswalk and if he is not moving
                    elif self.pedestrian_crossing and all(not x for x in self.pedestian_queue):
                        # self.pedestrian_crossing = False
                        rospy.logwarn("safe to cross!")
                        self.waiting_for_pedestrian = False
                        self.time_starting_crosswalk = rospy.Time.now()
                        # return
                    
                    #if neither return
                    else:
                        return
                    

                else:
                    self.previous_pedestrian_image = mid_section
                    velocity = Twist()
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.0
                    self.pub_cmd_vel.publish(velocity)
                    return
            #normal driving behaviour.
            
            try:
                ros_image = self.bridge.cv2_to_imgmsg(cv2_resized, encoding="bgr8")
                srv = rospy.ServiceProxy(self.section + '_service', ImageProcessor)
                resp = srv(ros_image)
                lin_pred, ang_pred = self.parse_result(resp.result)

                rospy.loginfo('Lin: '+ str(lin_pred)+' Ang: '+str(ang_pred))

                velocity = Twist()

                # this zeroes the linear velocity to prevent unwanted drivting behaviour  when the robot is supposed to be waiting at for example Yoda, or the truck.
                if lin_pred < 0.015 and (not self.has_not_reached_crosswalk and self.section == "Road") or self.section == "OffRoad":
                    lin_pred = 0.0
                    rospy.logwarn("zero'ed the vel")
                    

                velocity.linear.x = lin_pred * NN_OUTPUT_SCALAR
                velocity.angular.z = ang_pred * NN_OUTPUT_SCALAR

                if self.time_starting_crosswalk is not None and (rospy.Time.now() - self.time_starting_crosswalk).to_sec() < CROSSWALK_CROSSING_TIME:
                    velocity.linear.x = lin_pred * NN_OUTPUT_SCALAR * CROSSWALK_SPEED_UP
                    velocity.angular.z = ang_pred * NN_OUTPUT_SCALAR  * CROSSWALK_SPEED_UP
                    rospy.logwarn("speeding up to cross crosswalk!")

                self.pub_cmd_vel.publish(velocity)
            except Exception as e:
                print(f"Failed to call service: {e}")
                velocity = Twist()
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0

                self.pub_cmd_vel.publish(velocity)
        else:
            rospy.logdebug("auto not enabled")

        # Visualization (not required, but helpful):
        # annotated_img = cv2.resize(cv2_img, (IMG_WIDTH*2, IMG_HEIGHT*2), interpolation=cv2.INTER_LINEAR)
        # text_info = self.section if self.section else "Unknown section"
        # cv2.putText(
        #     annotated_img,
        #     text_info,
        #     (10, 30),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     1,
        #     (0, 0, 255),
        #     2,
        #     cv2.LINE_AA
        # )
        # cv2.imshow("feed", annotated_img)
        # cv2.waitKey(1)

    def track_section_callback(self, msg):
        """Sets self.section"""
        self.section = msg.data

    def auto_drive_callback(self, msg):
        """Enables or disables automatic CNN-based driving."""
        self.auto_enabled = msg.data

    def publish_status(self, event):
        """
        Timer callback or manual call to publish a status message summarizing
        the node's current state.
        """
        status_str = (
            f"DrivingNode -> auto_enabled:{self.auto_enabled}, section:{self.section}, "
            f"crosswalk_detected:{not self.has_not_reached_crosswalk}, waiting_for_pedestrian:{self.waiting_for_pedestrian}"
        )
        self.pub_status.publish(status_str)

    def parse_result(self, result_str):
        """
        Expects strings of the form:
        "<some_label> -> lin:<float>, ang:<float>"
        Returns a tuple: (label, lin_val, ang_val)
        label: e.g. "road"
        lin_val: float
        ang_val: float
        or None if parsing fails.
        """
        pattern = r'(.*?) -> lin:([-\d\.]+), ang:([-\d\.]+)'
        match = re.match(pattern, result_str.strip())
        if match:
            label = match.group(1).strip()         # e.g. "road"
            lin_val = float(match.group(2))        # e.g. 0.123
            ang_val = float(match.group(3))        # e.g. -0.456
            return lin_val, ang_val
        else:
            return None

# ------------------------------------------------------------------------------
#                              MAIN ROS NODE
# ------------------------------------------------------------------------------
def image_subscriber():
    """
    Initializes the node that handles CNN-based auto-driving by subscribing
    to image data and relevant signals to select the correct model.
    Also publishes a status string about internal states.
    """
    rospy.init_node('driving_code')

    driver = DrivingNode()

    # Subscribers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, driver.image_callback)
    rospy.Subscriber('/track_section', String, driver.track_section_callback)
    rospy.Subscriber('/auto', Bool, driver.auto_drive_callback)

    # Publish status on a timer
    rospy.Timer(rospy.Duration(1.0 / STATUS_PUBLISH_RATE_HZ), driver.publish_status)

    rospy.sleep(1)
    rospy.loginfo("CNN driving node initialized!")
    rospy.spin()

if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
