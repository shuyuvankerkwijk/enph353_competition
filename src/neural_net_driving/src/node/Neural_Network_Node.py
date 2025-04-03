#!/usr/bin/env python3
import rospy
import cv2
import tensorflow as tf
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

# ROS message types
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Our custom message/service
from neural_net_driving.msg import ImageWithID
from neural_net_driving.srv import ImageProcessor, ImageProcessorResponse

class ProcessingNode:
    def __init__(self):
        rospy.init_node('processing_node')

        # Create a CvBridge so we can convert sensor_msgs/Image <-> OpenCV
        self.bridge = CvBridge()

        rospy.logdebug("started")

        # Load your models
        self.model_Road = tf.keras.models.load_model('/home/fizzer/ros_ws/training_for_driving/Road/best_model.h5')
        self.model_Gravel = tf.keras.models.load_model('/home/fizzer/ros_ws/training_for_driving/Gravel/best_model.h5')
        self.model_OffRoad = tf.keras.models.load_model('/home/fizzer/ros_ws/training_for_driving/OffRoad/best_model.h5')
        self.model_ramp = tf.keras.models.load_model('/home/fizzer/ros_ws/training_for_driving/ramp/best_model.h5')
        # self.model_reading = tf.keras.models.load_model("/home/fizzer/ros_ws/training_for_reading/V4_model_stripped.h5")

        self.interpreter_reading = tf.lite.Interpreter(model_path="/home/fizzer/ros_ws/training_for_reading/V4_model_quantized.tflite")
        self.interpreter_reading.allocate_tensors()
        self.input_details_reading = self.interpreter_reading.get_input_details()
        self.output_details_reading = self.interpreter_reading.get_output_details()

        rospy.logdebug("loaded cnns")
        # Advertise 4 services
        self.srv1 = rospy.Service('/B1/Road_service', ImageProcessor, self.handle_road)
        self.srv2 = rospy.Service('/B1/Gravel_service', ImageProcessor, self.handle_gravel)
        self.srv3 = rospy.Service('/B1/OffRoad_service', ImageProcessor, self.handle_offroad)
        self.srv4 = rospy.Service('/B1/ramp_service', ImageProcessor, self.handle_ramp)

        rospy.logdebug("started services")

        # Subscriber to our custom ImageWithID topic
        self.sub = rospy.Subscriber('/read_input_images', ImageWithID, self.image_callback)

        # Publisher for results
        self.pub = rospy.Publisher('/read_image_results', String, queue_size=1000)

        # Buffer for storing images that need to be predicted using CNN
        self.image_buffer = []

        rospy.loginfo("Processing node started: 4 services + subscription to 'input_images'.")

    def handle_road(self, req):
        """Service callback for Road images -> output Twist command or something."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            return ImageProcessorResponse(result=f"Road service cv_bridge error: {e}")

        # Convert to normalized float32, shape = (H, W, 3)
        rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        normalized_img = rgb_img.astype(np.float32) / 255.0
        input_tensor = tf.expand_dims(normalized_img, axis=0)

        lin_pred, ang_pred = self.model_Road.predict(input_tensor)[0]
        return ImageProcessorResponse(result=f"road -> lin:{lin_pred:.3f}, ang:{ang_pred:.3f}")

    def handle_gravel(self, req):
        """Service callback for Gravel images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            return ImageProcessorResponse(result=f"Gravel service cv_bridge error: {e}")

         # Convert to normalized float32, shape = (H, W, 3)
        rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        normalized_img = rgb_img.astype(np.float32) / 255.0
        input_tensor = tf.expand_dims(normalized_img, axis=0)

        lin_pred, ang_pred = self.model_Gravel.predict(input_tensor)[0]
        return ImageProcessorResponse(result=f"gravel -> lin:{lin_pred:.3f}, ang:{ang_pred:.3f}")

    def handle_offroad(self, req):
        """Service callback for OffRoad images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            return ImageProcessorResponse(result=f"OffRoad service cv_bridge error: {e}")

        # Convert to normalized float32, shape = (H, W, 3)
        rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        normalized_img = rgb_img.astype(np.float32) / 255.0
        input_tensor = tf.expand_dims(normalized_img, axis=0)

        lin_pred, ang_pred = self.model_OffRoad.predict(input_tensor)[0]
        return ImageProcessorResponse(result=f"offroad -> lin:{lin_pred:.3f}, ang:{ang_pred:.3f}")

    def handle_ramp(self, req):
        """Service callback for ramp images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            return ImageProcessorResponse(result=f"Ramp service cv_bridge error: {e}")

         # Convert to normalized float32, shape = (H, W, 3)
        rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        normalized_img = rgb_img.astype(np.float32) / 255.0
        input_tensor = tf.expand_dims(normalized_img, axis=0)

        lin_pred, ang_pred = self.model_ramp.predict(input_tensor)[0]
        return ImageProcessorResponse(result=f"ramp -> lin:{lin_pred:.3f}, ang:{ang_pred:.3f}")

    def image_callback(self, msg):
        """Called whenever an `ImageWithID` arrives on `input_images` topic."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image) #encoding: 32FC1
        except CvBridgeError as e:
            rospy.logerr(f"Image callback cv_bridge error: {e}")
            return
        
        self.image_buffer.append([cv_image, msg.id])

    def process_image(self):
        if len(self.image_buffer) > 0:
            cv_image, id = self.image_buffer.pop(0)

            start = time.time()

            # Normalize + reshape input
            cv_image = cv_image.astype(np.float32) / 255.0
            cv_image = cv_image[..., None]      # (H, W, 1)
            cv_image = np.expand_dims(cv_image, 0)  # (1, H, W, 1)

            # Set input and run inference
            self.interpreter_reading.set_tensor(self.input_details_reading[0]['index'], cv_image)
            self.interpreter_reading.invoke()
            output = self.interpreter_reading.get_tensor(self.output_details_reading[0]['index'])

            # Get predicted label
            letter = int(np.argmax(output[0]))

            duration = time.time() - start
            rospy.loginfo(f"Inference time (TFLite): {duration:.4f} seconds")

            result_str = f"id: {id}, prediction: {letter}"
            self.pub.publish(result_str)
            
    # def process_image(self):
    #     if len(self.image_buffer) > 0:
    #         cv_image, id = self.image_buffer.pop(0)

    #         # rospy.loginfo(f"Received image with ID: {id}. Shape: {cv_image.shape}, DType: {cv_image.dtype}, Max: {np.max(cv_image)}") #TODO: shorten this, only as a check
    #         cv_image = cv_image.astype(np.float32) / 255.0
    #         cv_image = cv_image[..., None]      # (H, W, 1)
    #         cv_image = np.expand_dims(cv_image, 0)  # (1, H, W, 1)

    #         letter = np.argmax(self.model_reading.predict(cv_image)[0])

    #         result_str = f"id: {id}, prediction: {letter}"

    #         self.pub.publish(result_str)


if __name__ == '__main__':
    try:
        node = ProcessingNode()
        
        rate = rospy.Rate(20)  # 10Hz
        while not rospy.is_shutdown():
            node.process_image()
            rate.sleep()

        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
