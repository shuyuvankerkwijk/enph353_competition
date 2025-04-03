#! /usr/bin/env python3

import serial
import re
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# ----------------------------------------------------------------------
# Serial Configuration
# ----------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyUSB0"  # Change based on your setup
BAUD_RATE = 9600

# If your microcontroller message is "X:%f, Y:%f, R:True/False" then "R:" is correct
# Otherwise change to "r:" if the microcontroller sends r instead of R
JOYSTICK_REGEX = re.compile(r"X:(-?\d+\.\d+), Y:(-?\d+\.\d+), R:(True|False)")

# ----------------------------------------------------------------------
# Main Class for Data Collection and Joystick Handling
# ----------------------------------------------------------------------
class DataCollection:
    def __init__(self):
        """
        Initializes all required variables and a CvBridge to convert 
        ROS sensor_msgs/Image messages to OpenCV images.
        """
        self.x = 0.0
        self.y = 0.0
        self.r = False
        self.ser = None
        self.timeout = 0.1  # seconds to wait for a valid message

        # We'll use this buffer to store partial serial data between reads.
        self._static_buffer = ""

    def setup_serial(self):
        """Initialize the serial connection in non-blocking mode."""
        try:
            # timeout=0 => Non-blocking reads
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
            self.ser.flushInput()  # Clear any old data
            rospy.loginfo("Serial port initialized.")
        except serial.SerialException as e:
            rospy.logerr(f"Serial Error: {e}")
            self.ser = None

    def read_latest_joystick(self):
        if not self.ser:
            return None

        start_time = rospy.Time.now().to_sec()
        latest_valid_msg = None

        while rospy.Time.now().to_sec() - start_time < self.timeout:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1).decode("utf-8", errors="ignore")
                self._static_buffer += chunk

                while "\n" in self._static_buffer:
                    line, self._static_buffer = self._static_buffer.split("\n", 1)
                    line = line.strip()

                    match = JOYSTICK_REGEX.match(line)
                    if match:
                        x = float(match.group(1))
                        y = float(match.group(2))
                        r = (match.group(3) == "True")
                        # Don't return immediately, just store the latest
                        latest_valid_msg = (x, y, r)

            except Exception as ex:
                rospy.logwarn_once(f"Serial read error: {ex}")
                pass

        # After we've read everything we can for up to 'timeout', return the last found message
        if latest_valid_msg:
            self.x, self.y, self.r = latest_valid_msg
            return latest_valid_msg

        return None


# ----------------------------------------------------------------------
# Main ROS Node
# ----------------------------------------------------------------------
def joystick_publisher():
    rospy.init_node('joystick_teleop')  # Renamed for clarity

    data_saver = DataCollection()

    # Example publishers
    pub_cmd = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
    pub_record = rospy.Publisher('/record_topic', Bool, queue_size=1)

    # Give ROS some time to set up
    rospy.sleep(1)

    # Initialize the serial port
    data_saver.setup_serial()

    rate = rospy.Rate(20)  # 20Hz (~50ms loop)
    rospy.loginfo("Joystick node initialized!")

    while not rospy.is_shutdown():
        joystick_data = data_saver.read_latest_joystick()
        if joystick_data:
            x, y, r = joystick_data

            if r and y < 0:
                y=0

            # Create and publish a Twist message
            velocity = Twist()
            velocity.linear.x = y*0.75
            velocity.angular.z = -x

            pub_cmd.publish(velocity)
            pub_record.publish(Bool(r))

        rate.sleep()  # Sleep to maintain 20Hz loop

if __name__ == "__main__":
    try:
        joystick_publisher()
    except rospy.ROSInterruptException:
        pass