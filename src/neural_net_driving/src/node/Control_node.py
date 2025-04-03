#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from pynput import keyboard

# ------------------------------------------------------------------------------
#                                CONFIGURATION
# ------------------------------------------------------------------------------
KEY_STOP_ALL = 'x'        # Stop everything
KEY_TELEOP_TOGGLE = 't'   # Enable teleoperation
KEY_RESET = 'r'           # Reset to start
KEY_AUTO_DRIVE = 'a'      # Start CNN-based auto driving
KEY_START_COMPETITION = 's'# Start competition
KEY_PAUSE_DRIVING = 'p'   # Pause all driving
KEY_TELEPORT_GRAVEL = 'g' # Teleport to the Gravel section start
KEY_TELEPORT_OFFROAD = 'o'# Teleport to OffRoad section start
KEY_TELEPORT_RAMP = 'h'   # Teleport to the ramp/hill start

# Hard-coded positions/orientations for resetting or teleporting the robot
RESET_POS_X = 5.5
RESET_POS_Y = 2.5
RESET_POS_Z = 0.2
RESET_ORI_X = 0.0
RESET_ORI_Y = 0.0
RESET_ORI_Z = -1.57
RESET_ORI_W = 0.0

GRAVEL_POS_X = 0.5
GRAVEL_POS_Y = 0.0
GRAVEL_POS_Z = 0.2
GRAVEL_ORI_X = 0.0
GRAVEL_ORI_Y = 0.0
GRAVEL_ORI_Z = -1.57
GRAVEL_ORI_W = 0.0

OFFROAD_POS_X = -3.9
OFFROAD_POS_Y = 0.5
OFFROAD_POS_Z = 0.2
OFFROAD_ORI_X = 0.0
OFFROAD_ORI_Y = 0.0
OFFROAD_ORI_Z = -1.57
OFFROAD_ORI_W = 0.0

HILL_POS_X = -4.1
HILL_POS_Y = -2.3
HILL_POS_Z = 0.2
HILL_ORI_X = 0.0
HILL_ORI_Y = 0.0
HILL_ORI_Z = 0.0
HILL_ORI_W = 0.0

PUBLISH_RATE_HZ = 10   # Rate to publish teleop/auto states

# ------------------------------------------------------------------------------
#                         CLASS: KeyboardController
# ------------------------------------------------------------------------------
class KeyboardController:
    """
    Subscribes to keyboard input (via pynput) and controls the robot’s
    teleoperation and auto-driving states by publishing to relevant topics.
    """

    def __init__(self):
        """
        Initializes the node, publishers, and starts the keyboard listener.
        """
        rospy.init_node('keyboard_controller')

        # Publishers for toggling states and resetting
        self.pub_teleop = rospy.Publisher('/teleop', Bool, queue_size=1)
        self.pub_auto = rospy.Publisher('/auto', Bool, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=5)
        self.pub_reset = rospy.Publisher('/reset', String, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.pub_status = rospy.Publisher('/keyboard_controller/status', String, queue_size=1)

        self.sub_section = rospy.Subscriber('/track_section', String, self.section_callback)

        # Track teleop/auto states
        self.auto = False
        self.teleop = False

        self.section = None
        self.section_start_time = None

        # A zero Twist (for stopping motion)
        self.zero_vel = Twist()
        self.zero_vel.angular.z = 0
        self.zero_vel.linear.x = 0

        # Start the keyboard listener on another thread
        self.listener = keyboard.Listener(on_press=self.handle_keypress)
        self.listener.start()

    def section_callback(self, msg):
        if not msg.data == self.section:
            self.section_start_time = rospy.Time.now()
            self.section = msg.data

    def handle_keypress(self, key):
        """
        Called whenever a key is pressed. 
        Interprets certain keys to toggle teleop, auto mode, reset, 
        or set robot positions.
        """
        try:
            if key.char == KEY_STOP_ALL:   # 'x'
                self.teleop = False
                self.auto = False
                self.publish()
                self.publish_stop()
                self.pub_cmd_vel.publish(self.zero_vel)

            elif key.char == KEY_TELEOP_TOGGLE:  # 't'
                self.teleop = True
                self.auto = False
                self.publish()

            elif key.char == KEY_RESET:  # 'r'
                self.reset_position()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('Reset')
                self.publish()

            elif key.char == KEY_AUTO_DRIVE:  # 'a'
                self.teleop = False
                self.auto = True
                self.publish()

            elif key.char == KEY_START_COMPETITION:  # 's'
                self.teleop = False
                self.auto = True
                self.publish_start()
                self.publish()

            elif key.char == KEY_PAUSE_DRIVING:  # 'p'
                self.teleop = False
                self.auto = False
                self.publish()
                self.pub_cmd_vel.publish(self.zero_vel)

            elif key.char == KEY_TELEPORT_GRAVEL:  # 'g'
                self.teleport_gravel()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('Gravel')

            elif key.char == KEY_TELEPORT_OFFROAD:  # 'o'
                self.teleop = False
                self.auto = False
                self.teleport_offroad()
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('OffRoad')

            elif key.char == KEY_TELEPORT_RAMP:  # 'h'
                self.teleport_hill()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('ramp')

        except AttributeError:
            # Ignore special (non-character) keys like Shift, Ctrl, etc.
            pass

    def publish_start(self):
        """
        Publishes a signal indicating the start of the competition
        (e.g., time tracking).
        """
        self.pub_score.publish('TEAM4,unknown,0,AAAA')

    def publish_stop(self):
        """
        Publishes a signal indicating a complete stop or end of competition.
        """
        self.pub_score.publish('TEAM$,unknown,-1,AAAA')

    def publish(self):
        """
        Publishes the current auto and teleop states.
        """
        self.pub_auto.publish(Bool(self.auto))
        self.pub_teleop.publish(Bool(self.teleop))

    def teleport_gravel(self):
        """
        Moves the robot to the predefined gravel start position.
        """
        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = GRAVEL_POS_X
        msg.pose.position.y = GRAVEL_POS_Y
        msg.pose.position.z = GRAVEL_POS_Z
        msg.pose.orientation.x = GRAVEL_ORI_X
        msg.pose.orientation.y = GRAVEL_ORI_Y
        msg.pose.orientation.z = GRAVEL_ORI_Z
        msg.pose.orientation.w = GRAVEL_ORI_W

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            _resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def teleport_offroad(self):
        """
        Moves the robot to the predefined offroad start position.
        """
        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = OFFROAD_POS_X
        msg.pose.position.y = OFFROAD_POS_Y
        msg.pose.position.z = OFFROAD_POS_Z
        msg.pose.orientation.x = OFFROAD_ORI_X
        msg.pose.orientation.y = OFFROAD_ORI_Y
        msg.pose.orientation.z = OFFROAD_ORI_Z
        msg.pose.orientation.w = OFFROAD_ORI_W

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            _resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def teleport_hill(self):
        """
        Moves the robot to the predefined hill/ramp start position.
        """
        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = HILL_POS_X
        msg.pose.position.y = HILL_POS_Y
        msg.pose.position.z = HILL_POS_Z
        msg.pose.orientation.x = HILL_ORI_X
        msg.pose.orientation.y = HILL_ORI_Y
        msg.pose.orientation.z = HILL_ORI_Z
        msg.pose.orientation.w = HILL_ORI_W

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            _resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def reset_position(self):
        """
        Moves the robot back to its default starting position.
        """
        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = RESET_POS_X
        msg.pose.position.y = RESET_POS_Y
        msg.pose.position.z = RESET_POS_Z
        msg.pose.orientation.x = RESET_ORI_X
        msg.pose.orientation.y = RESET_ORI_Y
        msg.pose.orientation.z = RESET_ORI_Z
        msg.pose.orientation.w = RESET_ORI_W

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            _resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def run(self):
        """
        Continuously publish the current teleop/auto states and status info.
        """
        rate = rospy.Rate(PUBLISH_RATE_HZ)
        while not rospy.is_shutdown():
            # Publish teleop/auto states as usual
            self.publish()

            # Also publish a status string
            status_msg = String()
            status_msg.data = f"teleop={self.teleop}, auto={self.auto}"
            self.pub_status.publish(status_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
