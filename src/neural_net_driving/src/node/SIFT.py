#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
import cv2
import numpy as np
from collections import Counter
from tensorflow import keras



class sift_class:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_buffer = []
        self.section = None
        
        # adjustable parameters
        self.blue_threshold = 0.55
        self.scale_image = 0.4
        self.scale_template = 0.4
        self.min_match_count = 20
        
        # setup object detection
        self.sift = cv2.SIFT_create()
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.template_img = cv2.imread("../include/template_full.png", cv2.IMREAD_GRAYSCALE)
        self.template_img_small = cv2.resize(self.template_img, None, fx=self.scale_template, fy=self.scale_template, interpolation=cv2.INTER_AREA)
        self.template_kp, self.template_des = self.sift.detectAndCompute(self.template_img_small, None)
        self.model = keras.models.load_model("../include/model_run_4.h5")

        # empty objects for storing results
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

    def blue_threshold(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        return blue_mask

    def image_callback(self, msg):
        """
        Receives an Image from ROS, optionally displays it, and adds to processing buffer if blue threshold is met
        """
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        # calculate if blue content is above threshold
        small_img = cv2.resize(cv2_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_LINEAR)
        blue_mask = self.blue_threshold(small_img)
        white_pixels = np.count_nonzero(blue_mask == 255)
        percentage_blue = (white_pixels/blue_mask.size)*100
        
        # show image and percentage, comment out if not needed
        text = f"{percentage_blue:.2f}%"
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (10, 30)  # Bottom-left corner of text
        fontScale = 1
        color = (0, 0, 255)
        thickness = 2
        binary_bgr = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(binary_bgr, text, org, font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.imshow("feed", binary_bgr)
        cv2.waitKey(1)
        
        # if percentage > 0.55%, process the image
        if percentage_blue >= self.blue_threshold:
            self.image_buffer.append(cv2_img)

    # TODO: implement this function. it has to write to one of the sign-read 
    # lists use  a cnn to clasify the sign type and use another cnn to read the message
    #images are already black and white as per the mask.
    
    def process_a_image(self):
        if len(self.image_buffer)>0:
            image = self.image_buffer.pop()
            
            '''1. run SIFT and find homography'''
            
            try:
                # resize and run SIFT/FLANN
                image_small = cv2.resize(image, None, fx=self.scale_image, fy=self.scale_image, interpolation=cv2.INTER_AREA)
                image_gray = cv2.cvtColor(image_small, cv2.COLOR_BGR2GRAY)
                kp, des = self.sift.detectAndCompute(image_gray, None)
                matches = self.flann.knnMatch(self.template_des, des, k=2)
                good_matches = [m for m,n in matches if m.distance < 0.7*n.distance]
                
                # if not enough good matches, ignore image
                if len(good_matches) <= self.min_match_count:
                    return
                
                src_pts = np.float32([self.template_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                
                # warps the sign found in the image to the template size (1400x2400)
                S_image = np.array([self.scale_image, 0, 0], [0, self.scale_image, 0], [0, 0, 1])
                S_template_inv = np.array([1/self.scale_template, 0, 0], [0, 1/self.scale_template, 0], [0, 0, 1])
                H_S = S_template_inv @ H @ S_image
                warped_image = cv2.warpPerspective(image, H_S, (self.template_img.shape[1], self.template_img.shape[0]))
                    
            except Exception as e:
                print(f"Error in SIFT or homography: {e}")
                return
                   
            '''2. Threshold and crop'''
            
            try:
                # threshold for blue color
                blue_mask = self.blue_threshold(warped_image)
                
                # clean up cracks and gaps
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                mask_cleaned = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel, iterations = 2)
                
                # check if the warp was good and there is a complete border
                if (mask_cleaned[10, 10] == mask_cleaned[10, -10] == mask_cleaned[-10, 10] == mask_cleaned[-10, -10] == 255):
                    print("Border is complete")
                else:
                    print("Border is incomplete. Continuing anyways")
                
                # flood fill to get rid of the surrounding white frame
                flood_filled = mask_cleaned.copy()
                cv2.floodFill(flood_filled, None, (14, 14), 0) #starting at 14,14
                
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(flood_filled, connectivity=4)
                min_area = 300
                letter_boxes = []
                for label in range(1, num_labels): # skip label 0 (background)
                    x, y, w, h, area = stats[label]
                    if area > min_area:
                        letter_boxes.append((x,y,w,h))
                
                # crop image in half and sort labels
                img_height = flood_filled.shape[0]
                top_half = []
                bottom_half = []
                for box in letter_boxes:
                    x, y, w, h = box
                    center_y = y + h/2
                    if center_y < img_height/2:
                        top_half.append(box)
                    else:
                        bottom_half.append(box)
                top_half = sorted(top_half, key=lambda b: b[0])
                bottom_half = sorted(bottom_half, key=lambda b: b[0])
                
                # crop/pad all boxes to the same size
                def crop_or_pad_box(letter_boxes):
                    target_width = 200
                    target_height = 240
                    letter_crops = []
                    for (x, y, w, h) in letter_boxes:
                        crop = flood_filled[y:y+h, x:x+w]
                        if h > target_height or w > target_width:
                            crop = cv2.resize(crop, (min(w, target_width), min(h, target_height)), interpolation=cv2.INTER_AREA)
                            h, w = crop.shape 
                        pad_top = (target_height - h) // 2
                        pad_bottom = target_height - h - pad_top
                        pad_left = (target_width - w) // 2
                        pad_right = target_width - w - pad_left
                        padded_crop = cv2.copyMakeBorder(
                            crop,
                            top=pad_top, bottom=pad_bottom,
                            left=pad_left, right=pad_right,
                            borderType=cv2.BORDER_CONSTANT,
                            value=0
                        )
                        letter_crops.append(padded_crop)
                    return letter_crops
                top_half_crops = crop_or_pad_box(top_half)
                bottom_half_crops = crop_or_pad_box(bottom_half)
                
                # optionally plot the resulting letter boxes, comment out if not needed
                circled_image = cv2.cvtColor(flood_filled, cv2.COLOR_GRAY2BGR)
                for (x, y, w, h) in (top_half_crops + bottom_half_crops):
                    cv2.rectangle(circled_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow("feed", circled_image)
                cv2.waitKey(1)
                            
            except Exception as e:
                print(f"Error in thresholding or cropping: {e}")
                return
            
            '''3. CNN classification'''
            try:
                def predict_batch(batch):
                    batch_normalized = batch / 255.0  # shape: (N, H, W)
                    batch_input = np.expand_dims(batch_normalized, axis=-1) # shape: (N, H, W, 1)
                    batch_predictions = self.model.predict(batch_input)
                    pred_indices = np.argmax(batch_predictions, axis=1)
                    pred_letters = [chr(ord('A') + idx) for idx in pred_indices]
                    return pred_letters
                
                top_predictions = predict_batch(top_half_crops)
                bottom_predictions = predict_batch(bottom_half_crops)
                
                # plot the letters and predictions, can comment out if not needed
                top_images_with_labels = []
                for i in range(len(top_half_crops)):
                    img = top_half_crops[i]
                    img_bgr = cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
                    cv2.putText(img_bgr, top_predictions[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2, cv2.LINE_AA)
                    top_images_with_labels.append(img_bgr)
                    
                bottom_images_with_labels = []
                for i in range(len(bottom_half_crops)):
                    img = bottom_half_crops[i]
                    img_bgr = cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
                    cv2.putText(img_bgr, bottom_predictions[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2, cv2.LINE_AA)
                    bottom_images_with_labels.append(img_bgr)

                top_image = cv2.hconcat(top_images_with_labels)
                bottom_image = cv2.hconcat(bottom_images_with_labels)
                final_image = cv2.vconcat(top_image, bottom_image)

                cv2.imshow("Predicted Letters", final_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
            except Exception as e:
                print(f"Error in CNN: {e}")
                return
            
            '''4. Add to the correct list'''
            
            sign_title = "".join(top_predictions)
            message = "".join(bottom_predictions)

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
            else:
                print("Sign title not recognized")

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
