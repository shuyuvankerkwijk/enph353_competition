#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
import cv2
import numpy as np
from collections import Counter
from neural_net_driving.msg import ImageWithID
import re
from datetime import datetime

SIFT_IMAGE_HEIGHT = 1264
SIFT_IMAGE_WIDTH = 1536
SIFT_TEMPLATE_SCALE = 0.4
BLUE_THRESHOLD_PERCENTAGE = 0.55
MIN_MATCH_COUNT = 24
CATEGORY_LIST = ["SIZE", "CRIME", "TIME", "PLACE", "MOTIVE", "WEAPON", "BANDIT"]

class Sign:
    
    def __init__(self, num_top, num_bottom):
        self.top_letters = ['' for _ in range(num_top)]
        self.bottom_letters = ['' for _ in range(num_bottom)]
        self.done = False
        self.top_word = None
        self.bottom_word = None
        self.score = None

    def place(self, type_char, letter_index, prediction):
        """
        Places a predicted letter in its correct slot and checks for completion
        """
        if type_char == 't':
            self.top_letters[letter_index] = prediction

        elif type_char == 'b':
            self.bottom_letters[letter_index] = prediction

        self.check_completion()

    def check_completion(self):
        if ('' not in self.top_letters) and ('' not in self.bottom_letters):
            self.done = True
            self.top_word, self.score = self.find_closest_category()
            self.bottom_word = ''.join(self.bottom_letters).upper()
        
    def find_closest_category(self):
        """
        Finds the closest word in category_list to `word`
        using Levenshtein-like edit distance.

        Returns:
            best_match: str — closest match
            distance: int — number of characters different
        """
        def edit_distance(a, b):
            # Dynamic programming Levenshtein distance
            dp = [[0]*(len(b)+1) for _ in range(len(a)+1)]

            for i in range(len(a)+1):
                for j in range(len(b)+1):
                    if i == 0:
                        dp[i][j] = j
                    elif j == 0:
                        dp[i][j] = i
                    elif a[i-1] == b[j-1]:
                        dp[i][j] = dp[i-1][j-1]
                    else:
                        dp[i][j] = 1 + min(
                            dp[i-1][j],    # deletion
                            dp[i][j-1],    # insertion
                            dp[i-1][j-1]   # substitution
                        )
            return dp[len(a)][len(b)]

        best_match = None
        best_distance = float('inf')

        for category in CATEGORY_LIST:
            dist = edit_distance(''.join(self.top_letters).upper(), category.upper())
            if dist < best_distance:
                best_match = category
                best_distance = dist

        return best_match, best_distance

class Concensus:
    def __init__(self):
        self.messages = []
        self.last_updated_time = None
        self.broadcast = False
        self.threshold = 1 # valid messages can only have 0 or 1 mistakes

    def add(self, message, score):
        if score <= self.threshold:
            self.last_updated_time = datetime.now()
            self.messages.append(message)

    def attempt_concensus(self):
        if self.last_updated_time is not None:
            if (self.last_updated_time - datetime.now()).seconds >= 5:
                message = self.majority_vote()
                return message
        return None

    def majority_vote(self): #TODO: change this
        max_len = max(len(s) for s in self.messages)
        padded = [s.ljust(max_len) for s in self.messages]  # pad with spaces
        result = ""
        for i in range(max_len):
            chars = [s[i] for s in padded]
            most_common = Counter(chars).most_common(1)[0][0]
            result += most_common
        return result.strip()  # strip padding
    
class sift_class:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_buffer = []
        self.section = None

        # topics
        self.pub_status = rospy.Publisher('/SIFT_node/status', String, queue_size=1)
        self.pub_read = rospy.Publisher('/read_input_images', ImageWithID, queue_size=1000)
        self.sub_read = rospy.Subscriber('/read_image_results', String, self.prediction_callback)
        self.pub_reads = rospy.Publisher('/score_tracker', String, queue_size=5)
        
        # setup object detection
        self.sift = cv2.SIFT_create()
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.template_img = cv2.imread("/home/fizzer/ros_ws/training_for_reading/template_full.png", cv2.IMREAD_GRAYSCALE)
        self.template_img_small = cv2.resize(self.template_img, None, fx=SIFT_TEMPLATE_SCALE, fy=SIFT_TEMPLATE_SCALE, interpolation=cv2.INTER_AREA)
        self.template_kp, self.template_des = self.sift.detectAndCompute(self.template_img_small, None)

        # empty objects for storing results
        self.sign_index = 0
        self.signs = {}
        self.consensuses = {}

        for i, category in enumerate(CATEGORY_LIST):
            self.consensuses[category] = Concensus()


    def blue_threshold(self, image):
        B, G, R = cv2.split(image)
        red_green_close = np.abs(R.astype(np.int16) - G.astype(np.int16)) <= 10
        blue_dominant = (((B.astype(np.int16) - R.astype(np.int16)) >= 90) & ((B.astype(np.int16) - R.astype(np.int16)) <= 110)) & \
                        (((B.astype(np.int16) - G.astype(np.int16)) >= 90) & ((B.astype(np.int16) - G.astype(np.int16)) <= 110))

        blue_mask = red_green_close & blue_dominant
        result = np.zeros_like(B, dtype=np.uint8)
        result[blue_mask] = 255

        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([130, 255, 255])
        # blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # return blue_mask
        return result

    def image_callback(self, msg):
        """
        Receives an Image from ROS, optionally displays it, and adds to processing buffer if blue threshold is met
        """

        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        # calculate if blue content is above threshold
        blue_mask = self.blue_threshold(cv2_img)
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
        binary_bgr = cv2.resize(binary_bgr, (768, 632), interpolation=cv2.INTER_LINEAR)
        cv2.putText(binary_bgr, text, org, font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.imshow("SIFT", binary_bgr)
        cv2.waitKey(1)

        # if percentage > 0.55%, process the image
        if percentage_blue >= BLUE_THRESHOLD_PERCENTAGE:
            rospy.loginfo(f"Appended image with threshold {percentage_blue}")
            self.pub_status.publish(String(""))
            self.image_buffer.append(cv2_img)

    def process_image(self):
        if len(self.image_buffer) == 0:
            return

        image = self.image_buffer.pop()

        try:
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kp, des = self.sift.detectAndCompute(image_gray, None)

            if des is None or len(kp) == 0:
                rospy.logwarn("No keypoints/descriptors found in the input image. Skipping.")
                return

            matches = self.flann.knnMatch(des, self.template_des, k=2)
            good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]

            if len(good_matches) <= MIN_MATCH_COUNT:
                # rospy.logwarn(f"Not enough matches ({len(good_matches)}) to compute homography. Skipping image.")
                return

            src_pts = np.float32([kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([self.template_kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if H is None:
                # rospy.logwarn("Homography could not be computed. Skipping image.")
                return
            
             # warps the sign found in the image to the template size (1400x2400)
            S_image = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])

            S_template_inv = np.array([
                [1/SIFT_TEMPLATE_SCALE, 0, 0],
                [0, 1/SIFT_TEMPLATE_SCALE, 0],
                [0, 0, 1]
            ])
            H_S = S_template_inv @ H @ S_image

            warped_image = cv2.warpPerspective(image, H_S, (self.template_img.shape[1], self.template_img.shape[0]))

        except Exception as e:
            rospy.logerr(f"Exception during SIFT/homography: {e}")
            return
        
        # cv2.imshow("warped_image", warped_image)
        # cv2.waitKey(1)
        # cv2.imwrite(f"/home/fizzer/ros_ws/training_for_driving/debug_warped_image{warped_image[40][40]}.png", warped_image)

        try:
            blue_mask = self.blue_threshold(warped_image)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask_cleaned = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            border_check = [mask_cleaned[10, 10], mask_cleaned[10, -10], mask_cleaned[-10, 10], mask_cleaned[-10, -10]]

            #TODO
            # if all(val == 255 for val in border_check):
            #     rospy.loginfo("Border appears complete.")
            # else:
            #     rospy.logwarn("Border is incomplete. Continuing anyway...")

            flood_filled = mask_cleaned.copy()
            cv2.floodFill(flood_filled, None, (14, 14), 0)

            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(flood_filled, connectivity=4)

            letter_boxes = []
            for i in range(1, num_labels):
                x, y, w, h, area = stats[i]
                if area > 300:
                    letter_boxes.append((x, y, w, h))

            img_height = flood_filled.shape[0]
            top_half, bottom_half = [], []
            for box in letter_boxes:
                x, y, w, h = box
                if y + h / 2 < img_height / 2:
                    top_half.append(box)
                else:
                    bottom_half.append(box)

            top_half = sorted(top_half, key=lambda b: b[0])
            bottom_half = sorted(bottom_half, key=lambda b: b[0])

            def crop_or_pad_box(letter_boxes):
                target_width = 200
                target_height = 240
                crops = []
                for (x, y, w, h) in letter_boxes:
                    crop = flood_filled[y:y + h, x:x + w]
                    crop = cv2.resize(crop, (min(w, target_width), min(h, target_height)), interpolation=cv2.INTER_AREA)
                    h2, w2 = crop.shape
                    pad_top = (target_height - h2) // 2
                    pad_bottom = target_height - h2 - pad_top
                    pad_left = (target_width - w2) // 2
                    pad_right = target_width - w2 - pad_left
                    padded = cv2.copyMakeBorder(
                        crop, pad_top, pad_bottom, pad_left, pad_right,
                        borderType=cv2.BORDER_CONSTANT, value=0
                    )
                    crops.append(padded)
                return crops

            top_half_crops = crop_or_pad_box(top_half)
            bottom_half_crops = crop_or_pad_box(bottom_half)

        except Exception as e:
            rospy.logerr(f"Exception during thresholding/cropping: {e}")
            return
    
        # circled_image = cv2.cvtColor(flood_filled, cv2.COLOR_GRAY2BGR)
        # for (x, y, w, h) in (top_half + bottom_half):
        #     cv2.rectangle(circled_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # cv2.imwrite(f"/home/fizzer/ros_ws/training_for_driving/debug_circled_image{warped_image[40][40][2]}.png", circled_image)
        # rospy.loginfo("Saved wcircledarped image")
        # cv2.imshow("circled_image", circled_image)
        # cv2.waitKey(1)
        
        try:
            self.sign_index += 1
            self.signs[int(self.sign_index)] = Sign(len(top_half_crops), len(bottom_half_crops))

            for i, image in enumerate(top_half_crops):
                image = cv2.resize(image, (40, 48))
                ros_image = self.bridge.cv2_to_imgmsg(image) #, encoding='32FC1'
                msg = ImageWithID()
                msg.id = f"{self.sign_index}t{i}"
                msg.image = ros_image
                self.pub_read.publish(msg)

            for i, image in enumerate(bottom_half_crops):
                image = cv2.resize(image, (40, 48))
                ros_image = self.bridge.cv2_to_imgmsg(image) #, encoding='32FC1'
                msg = ImageWithID()
                msg.id = f"{self.sign_index}b{i}"
                msg.image = ros_image
                self.pub_read.publish(msg)

        except Exception as e:
            rospy.logerr(f"Exception during CNN classification/publishing: {e}")

    
    def parse_result_string(self, result_str):
        """
        Parses strings like: "id: 42t7, prediction: 26"
        Returns:
            sign_counter (int),
            type_char (str),
            i (int),
            prediction (int)
        """
        pattern = r'^id:\s*(\d+)([a-zA-Z])(\d+),\s*prediction:\s*(-?\d+)$'
        match = re.match(pattern, result_str.strip())
        if match:
            sign_index = int(match.group(1))
            type_char = match.group(2)
            letter_index = int(match.group(3))
            prediction = int(match.group(4))
            return sign_index, type_char, letter_index, prediction
        else:
            return None, None, None, None


    def prediction_callback(self, result_string):

        try:
            sign_index, type_char, letter_index, prediction = self.parse_result_string(result_string.data)
            prediction = chr(ord('A') + prediction) # convert to letter

            # rospy.loginfo(f"sidx: {sign_index}, typ char: {type_char}, lidx: {letter_index}, prediction: {prediction}")

            sign = self.signs[int(sign_index)]
            sign.place(type_char, letter_index, prediction)

            if sign.done == True: 
                # rospy.loginfo(f"sign {sign_index} is DONE, with score {sign.score} and letters {str(sign.top_letters)} and {sign.top_word}, {sign.bottom_word}")
                self.consensuses[sign.top_word].add(sign.bottom_word, sign.score)
                self.signs.pop(int(sign_index))
         
        except Exception as e:
            rospy.logerr(f"Exception during prediction callback: {e}")

    def attempt_concensus(self):
        for category, concensus in self.consensuses.items():
            message = concensus.attempt_concensus()
            if message is not None:
                if concensus.broadcast == False:
                    rospy.logerr("Determined: " + category + ": " + message)
                    concensus.broadcast = True

                # if category == "SIZE":
                #     self.pub_reads.publish(f"SIZE {message}")
                # elif category == "CRIME":
                #     self.pub_reads.publish(f"CRIME {message}")
                # elif category == "TIME":
                #     self.pub_reads.publish(f"TIME {message}")
                # elif category == "PLACE":
                #     self.pub_reads.publish(f"PLACE {message}")
                # elif category == "MOTIVE":
                #     self.pub_reads.publish(f"MOTIVE {message}")
                # elif category == "WEAPON":
                #     self.pub_reads.publish(f"WEAPON {message}")
                # elif category == "BANDIT":
                #     self.pub_reads.publish(f"BANDIT {message}")

    def track_section_callback(self, msg):
        self.section = msg.data
    
    def results(self, msg):
        return


def image_subscriber():
    rospy.init_node('SIFT_node')  # Renamed for clarity

    sift = sift_class()

    # Example publishers
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, sift.image_callback)
    rospy.Subscriber('/track_section', String, sift.track_section_callback)
    rospy.Subscriber('/read_image_results', String, sift.results)

    # Give ROS some time to set up
    rospy.sleep(1)

    # rospy.loginfo("SIFT node initialized!")

    rate = rospy.Rate(5)  # 5Hz 
    while not rospy.is_shutdown():
        sift.process_image()
        sift.attempt_concensus()
        rate.sleep()
 


if __name__ == "__main__":
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass





    