#! /usr/bin/env python3

import serial
import re
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

# ------------------------------------------------------------------------------
#                                CONFIGURATION
# ------------------------------------------------------------------------------
JOYSTICK_SERIAL_PORT = "/dev/ttyUSB0"         # Serial port connected to microcontroller
JOYSTICK_BAUD_RATE = 9600                     # Baud rate for serial connection
JOYSTICK_READ_TIMEOUT_SEC = 0.1               # Time (s) to attempt reading joystick data
JOYSTICK_SPEED_SCALE = 0.75                   # Scale factor for linear velocity
JOYSTICK_ANGULAR_SCALE = 1.0                  # Scale factor for angular velocity
PUBLISH_RATE_HZ = 20                          # Frequency to publish cmd_vel messages

# Regex that matches microcontroller’s joystick message format, e.g.:
# "X:0.12, Y:-0.40, R:True" or "... R:False"
JOYSTICK_REGEX = re.compile(r"X:(-?\d+\.\d+), Y:(-?\d+\.\d+), R:(True|False)")

# ------------------------------------------------------------------------------
#                           CLASS: JoystickDataHandler
# ------------------------------------------------------------------------------
class JoystickDataHandler:
    """
    Reads joystick data over a serial connection and publishes
    velocity commands if teleoperation is enabled.
    
    Attributes:
        joystick_x (float):     Most recent joystick X-axis value.
        joystick_y (float):     Most recent joystick Y-axis value.
        joystick_button (bool): Most recent button state on the joystick.
        ser (serial.Serial):    Serial connection instance.
        buffer_fragment (str):  Temporary buffer for partial serial lines.
        teleop_enabled (bool):  Determines whether to interpret and publish moves.
    """

    def __init__(self):
        """
        Initialize internal state for reading joystick data.
        """
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.joystick_button = False

        self.ser = None
        self.buffer_fragment = ""
        self.teleop_enabled = False

    def setup_serial_connection(self):
        """
        Attempt to open the serial connection in non-blocking mode.
        If unsuccessful, self.ser remains None and reading is skipped.
        """
        try:
            self.ser = serial.Serial(
                JOYSTICK_SERIAL_PORT,
                JOYSTICK_BAUD_RATE,
                timeout=0  # non-blocking
            )
            self.ser.flushInput()
            rospy.loginfo("Serial port initialized.")
        except serial.SerialException as e:
            rospy.logerr(f"Serial Error: {e}")
            self.ser = None

    def read_latest_joystick(self):
        """
        Continuously reads from the serial buffer for up to JOYSTICK_READ_TIMEOUT_SEC.
        Stores the last valid parsed joystick line in (joystick_x, joystick_y, joystick_button).

        Returns:
            tuple or None: (x, y, btn_state) if a valid reading was found, else None.
        """
        if not self.ser:
            return None

        start_time = rospy.Time.now().to_sec()
        latest_valid_msg = None

        while rospy.Time.now().to_sec() - start_time < JOYSTICK_READ_TIMEOUT_SEC:
            try:
                # Read available data from the serial buffer
                chunk = self.ser.read(self.ser.in_waiting or 1).decode("utf-8", errors="ignore")
                self.buffer_fragment += chunk

                # Process any complete lines
                while "\n" in self.buffer_fragment:
                    line, self.buffer_fragment = self.buffer_fragment.split("\n", 1)
                    line = line.strip()

                    match = JOYSTICK_REGEX.match(line)
                    if match:
                        x_val = float(match.group(1))
                        y_val = float(match.group(2))
                        button_state = (match.group(3) == "True")
                        # Keep storing the last recognized reading
                        latest_valid_msg = (x_val, y_val, button_state)

            except Exception as ex:
                rospy.logwarn_once(f"Serial read error: {ex}")
                pass

        # After reading up to the timeout, use the final valid message
        if latest_valid_msg:
            self.joystick_x, self.joystick_y, self.joystick_button = latest_valid_msg
            return latest_valid_msg

        return None

    def teleop_state_callback(self, msg):
        """
        Callback for /teleop subscriber.
        Enables or disables joystick teleoperation.
        """
        self.teleop_enabled = msg.data

# ------------------------------------------------------------------------------
#                               MAIN ROS NODE
# ------------------------------------------------------------------------------
def joystick_publisher():
    """
    Initializes the ROS node to read joystick data from a microcontroller
    and publish geometry_msgs/Twist commands if teleoperation is enabled.
    Also publishes node status on /joystick_teleop/status.
    """
    rospy.init_node('joystick_teleop')

    # Instantiate our joystick handler
    joystick_handler = JoystickDataHandler()

    # Publishers and Subscribers
    pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
    pub_status = rospy.Publisher('/joystick_teleop/status', String, queue_size=1)
    rospy.Subscriber('/teleop', Bool, joystick_handler.teleop_state_callback)

    rospy.sleep(1)  # Allow ROS time to set up

    # Attempt to connect to the serial joystick
    joystick_handler.setup_serial_connection()

    rate = rospy.Rate(PUBLISH_RATE_HZ)
    rospy.loginfo("Joystick teleop node initialized!")

    while not rospy.is_shutdown():
        joystick_data = joystick_handler.read_latest_joystick()
        if joystick_data and joystick_handler.teleop_enabled:
            x_val, y_val, _ = joystick_data
            velocity_msg = Twist()
            velocity_msg.linear.x = y_val * JOYSTICK_SPEED_SCALE
            velocity_msg.angular.z = -x_val * JOYSTICK_ANGULAR_SCALE
            pub_cmd_vel.publish(velocity_msg)

        # Publish status
        status_msg = String()
        status_msg.data = (
            f"teleop_enabled={joystick_handler.teleop_enabled}, "
            f"joystick_x={joystick_handler.joystick_x:.3f}, "
            f"joystick_y={joystick_handler.joystick_y:.3f}, "
            f"joystick_button={joystick_handler.joystick_button}"
        )
        pub_status.publish(status_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        joystick_publisher()
    except rospy.ROSInterruptException:
        pass
