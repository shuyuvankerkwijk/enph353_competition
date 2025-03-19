#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
import keyboard
from gazebo_msgs.msg import ModelState


class KeyboardController:
    def __init__(self):
        rospy.init_node('keyboard_controller', anonymous=True)
        self.pub_teleop = rospy.Publisher('/teleop', Bool, queue_size=1)
        self.pub_auto = rospy.Publisher('/auto', Bool, queue_size=1)
        self.pub_score = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.auto = False
        self.teleop = False

    def parse(self, key):
        if key.name == 'x':#stop everything not timing 
            self.teleop = False
            self.auto = False
            self.publish()
            self.publish_stop()
        elif key.name == 't':#teleoperation
            self.teleop = True
            self.auto = False
            self.publish()
        elif key.name == 'r':#reset go to start
            self.reset()
            self.teleop = False
            self.auto = False
            self.publish()
        elif key.name == 'a':#auto cnn driving
            self.teleop = False
            self.auto = True
            self.publish()
        elif key.name == 's':#start for comp start timing and auto
            self.teleop = False
            self.auto = True
            self.publish_start()
            self.publish()
        elif key.name == 'p':#pause driving
            self.teleop = False
            self.auto = False
            self.publish()


    def publish_start(self):
        self.pub_score.publish('gadget, unknown, 0, AAAA')
        return
    
    def publish_stop(self):
        self.pub_score.publish('gadget, unknown, -1, AAAA')
        return

    def publish(self):
        self.pub_auto.publish(self.auto)
        self.pub_teleop.publish(self.teleop)
        return
    
    def reset():
        msg = ModelState()
        msg.model_name = 'B1'

        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

        return

    def run(self):
        # Set up keyboard listener
        keyboard.on_press(self.parse)
        
        # Publish at 10Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        keyboard.unhook_all()  # Cleanup