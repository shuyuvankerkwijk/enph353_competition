#! /usr/bin/env python3


import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tensorflow as tf
import numpy as np

IMG_HEIGHT = 316
IMG_WIDTH = 384

class CNN_driving_class:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.section = None
        self.bridge = CvBridge()
        self.model = None
        self.run = False
        self.model_Road = tf.keras.models.load_model('/home/fizzer/images_for_cnn_training/Road/best_model.h5')
        self.model_Gravel = tf.keras.models.load_model('/home/fizzer/images_for_cnn_training/Gravel/best_model.h5')
        self.model_OffRoad = tf.keras.models.load_model('/home/fizzer/images_for_cnn_training/OffRoad/best_model.h5')
        self.model_ramp = tf.keras.models.load_model('/home/fizzer/images_for_cnn_training/ramp/best_model.h5')

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        if self.run:
            if self.model is not None:
                cv2_img = cv2.resize(cv2_img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_LINEAR)
                rgb_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
                normalized_img = rgb_img.astype(np.float32) / 255.0
                input_tensor = tf.expand_dims(normalized_img, axis=0)
                lin_pred, ang_pred = self.model.predict(input_tensor)[0]
                rospy.loginfo('Lin: '+ str(lin_pred)+' Ang: '+str(ang_pred))

                velocity = Twist()
                velocity.linear.x = lin_pred
                velocity.angular.z = ang_pred

                self.pub_cmd_vel.publish(velocity)

        img = cv2.resize(cv2_img,(IMG_WIDTH*2, IMG_HEIGHT*2), interpolation=cv2.INTER_LINEAR)
        text = self.section
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (10, 30)  # Bottom-left corner of text
        fontScale = 1
        color = (0, 0, 255)  # Green if recording, red otherwise
        thickness = 2

        # Draw text on the image
        cv2.putText(img, text, org, font, fontScale, color, thickness, cv2.LINE_AA) 
        cv2.imshow("feed", img)
        cv2.waitKey(1)

    def track_section_callback(self, msg):
        if not msg.data == self.section:
            self.section = msg.data
            if self.section == "Road":
                self.model = self.model_Road
            elif self.section == "Gravel":
                self.model = self.model_Gravel
            elif self.section == "OffRoad":
                self.model = self.model_OffRoad
            elif self.section == "ramp":
                self.model = self.model_ramp
            else:
                rospy.logerr("Invalid_section")

    def auto(self, msg):
        self.run = msg.data


def image_subscriber():
    rospy.init_node('driving_code')  # Renamed for clarity

    driver = CNN_driving_class()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, driver.image_callback)
    rospy.Subscriber('/track_section', String, driver.track_section_callback)
    rospy.Subscriber('/auto', Bool, driver.auto)


    # Give ROS some time to set up
    rospy.sleep(1)
    rospy.loginfo("CNN driving node initialized!")
    rospy.spin()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass