#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import threading

# Store the latest status from each node in a dictionary
latest_status = {
    "joystick_teleop": "No data",
    "control_node": "No data",
    "driving_code": "No data",
    "section_detector": "No data",
    "sift_node": "",
}

status_lock = threading.Lock()

def joystick_status_cb(msg):
    global latest_status
    with status_lock:
        latest_status["joystick_teleop"] = msg.data

def control_status_cb(msg):
    global latest_status
    with status_lock:
        latest_status["control_node"] = msg.data

def driving_status_cb(msg):
    global latest_status
    with status_lock:
        latest_status["driving_code"] = msg.data

def section_det_status_cb(msg):
    global latest_status
    with status_lock:
        latest_status["section_detector"] = msg.data

def sift_status_cb(msg):
    global latest_status
    with status_lock:
        latest_status["sift_node"] = latest_status["sift_node"] + ", " + msg.data

def display_dashboard():
    """
    Runs in a loop and shows a simple GUI (OpenCV window) with the latest statuses.
    """
    rate = rospy.Rate(5)  # 5 Hz update rate
    while not rospy.is_shutdown():
        # Create a black image for displaying text
        dashboard = np.zeros((300, 800, 3), dtype=np.uint8)

        # Write each node's status on its own line
        y_offset = 20
        with status_lock:
            for node_name, status_text in latest_status.items():
                cv2.putText(
                    dashboard,
                    f"{node_name}: {status_text}",
                    (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2
                )
                y_offset += 40

        # Show our "Dashboard" window
        cv2.imshow("Dashboard", dashboard)
        cv2.waitKey(1)

        rate.sleep()

def dashboard_node():
    """
    Initializes a ROS node that subscribes to all status topics and displays them
    in a dedicated OpenCV GUI window called "Dashboard."
    """
    rospy.init_node("dashboard_node", anonymous=True)

    # Subscribe to each node's status topic
    rospy.Subscriber("/joystick_teleop/status", String, joystick_status_cb)
    rospy.Subscriber("/control_node/status", String, control_status_cb)
    rospy.Subscriber("/driving_code/status", String, driving_status_cb)
    rospy.Subscriber("/section_detector/status", String, section_det_status_cb)
    rospy.Subscriber('/SIFT_node/status', String, sift_status_cb)

    # Launch the display loop (this blocks until ROS shutdown)
    display_dashboard()

if __name__ == "__main__":
    try:
        dashboard_node()
    except rospy.ROSInterruptException:
        pass
