#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from pynput import keyboard


class KeyboardController:

    def __init__(self):
        rospy.init_node('keyboard_controller')
        self.pub_teleop = rospy.Publisher('/teleop', Bool, queue_size=1)
        self.pub_auto = rospy.Publisher('/auto', Bool, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.pub_reset = rospy.Publisher('/reset', String, queue_size=1)  # TODO: Make one state string topic
        self.pub_cmd_vel = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.auto = False
        self.teleop = False
        self.zero_vel = Twist()

        # Start the keyboard listener in a separate thread
        self.listener = keyboard.Listener(on_press=self.parse)
        self.listener.start()

    def parse(self, key):
        try:
            if key.char == 'x':  # Stop everything
                self.teleop = False
                self.auto = False
                self.publish()
                self.publish_stop()
                self.pub_cmd_vel.publish(self.zero_vel)
            elif key.char == 't':  # Teleoperation
                self.teleop = True
                self.auto = False
                self.publish()
            elif key.char == 'r':  # Reset to start
                self.reset()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('Reset')
                self.publish()
            elif key.char == 'a':  # Auto CNN driving
                self.teleop = False
                self.auto = True
                self.publish()
            elif key.char == 's':  # Start competition (timing + auto)
                self.teleop = False
                self.auto = True
                self.publish_start()
                self.publish()
            elif key.char == 'p':  # Pause driving
                self.teleop = False
                self.auto = False
                self.publish()
                self.pub_cmd_vel.publish(self.zero_vel)
            elif key.char == 'g': #teleport to gravel start just slightly past view of pink line
                self.teleport_gravel()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('Gravel')
            elif key.char == 'o': #teleport to offroard start
                self.teleop = False
                self.auto = False
                self.teleport_offroad()
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('OffRoad')
            elif key.char == 'h':  #teleport to hill start
                self.teleport_hill()
                self.teleop = False
                self.auto = False
                self.pub_cmd_vel.publish(self.zero_vel)
                self.pub_reset.publish('ramp')
        except AttributeError:
            # Ignore special keys (Shift, Ctrl, etc.)
            pass

    def publish_start(self):
        self.pub_score.publish('gadget, unknown, 0, AAAA')

    def publish_stop(self):
        self.pub_score.publish('gadget, unknown, -1, AAAA')

    def publish(self):
        self.pub_auto.publish(Bool(self.auto))
        self.pub_teleop.publish(Bool(self.teleop))

    def teleport_gravel(self):

        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0
        msg.pose.position.z = 0.2
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = -1.57
        msg.pose.orientation.w = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")


    def teleport_offroad(self):

        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = -3.9
        msg.pose.position.y = 0.5
        msg.pose.position.z = 0.2
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = -1.57
        msg.pose.orientation.w = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def teleport_hill(self):

        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = -4.1
        msg.pose.position.y = -2.3
        msg.pose.position.z = 0.2
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def reset(self):

        msg = ModelState()
        msg.model_name = 'B1'
        msg.pose.position.x = 5.5
        msg.pose.position.y = 2.5
        msg.pose.position.z = 0.2
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = -1.57
        msg.pose.orientation.w = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call to set_model_state failed")

    def run(self):
        rate = rospy.Rate(10)  # 10Hz publishing rate
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
