#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import *

class JoyCtrl:
    def __init__(self):
        rospy.init_node('joy_node')
        self.diff_rotate_axis = rospy.get_param('~diff_rotate_axis', 0)
        self.diff_angle_axis = rospy.get_param('~diff_angle_axis', 1)
        self.twist_axis = rospy.get_param('~twist_axis', 2)
        self.activate_button = rospy.get_param('~activate_button', 11)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.diff_angle_pub = rospy.Publisher('wrist_diff/desired_angle', Float32)
        self.diff_rotate_pub = rospy.Publisher('wrist_diff/desired_rotation', Float32)
        self.twist_pub = rospy.Publisher('wrist_diff/desired_twist', Float32)

    def joy_cb(self, data):
        axes = data.axes
        buttons = data.buttons
        # rospy.loginfo(rospy.get_name() + ': %.02f %.02f %d' % (axes[self.diff_rotate_axis], axes[self.diff_angle_axis], buttons[self.activate_button]))
        if buttons[self.activate_button] > 0:
            diff_rotate_val = axes[self.diff_rotate_axis]
            diff_angle_val = axes[self.diff_angle_axis]
            twist_angle_val = axes[self.twist_axis]
            pi = 3.1415926535

            diff_rotate_ang = diff_rotate_val * pi
            diff_angle_ang = diff_angle_val * pi / 2
            twist_ang = twist_angle_val * 2

            self.diff_rotate_pub.publish(data=diff_rotate_ang)
            self.diff_angle_pub.publish(data=diff_angle_ang)
            self.twist_pub.publish(data=twist_ang)

if __name__ == '__main__':
    try:
        joy = JoyCtrl()
        rospy.spin()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass
