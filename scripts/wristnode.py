#!/usr/bin/env python
import roslib
roslib.load_manifest('dynamixel_controllers')

import rospy
from pid_controller import *
from std_msgs.msg import *
from wrist_node.msg import RawPosition, Data
from dynamixel_controllers.srv import SetSpeed

class WristNode:
    def __init__(self):
        rospy.init_node('wrist_node')

        # get parameters
        self.diff_left_name = rospy.get_param('~diff_left', 'diff_left_controller')
        self.diff_right_name = rospy.get_param('~diff_right', 'diff_right_controller')
        self.wrist_rotate_name = rospy.get_param('~wrist_rotate', 'base_controller')

        rotate_Kp = rospy.get_param('~rotate_Kp', 2)
        rotate_Ki = rospy.get_param('~rotate_Ki', 0)
        rotate_Kd = rospy.get_param('~rotate_Kd', 0)

        angle_Kp = rospy.get_param('~angle_Kp', 5)
        angle_Ki = rospy.get_param('~angle_Ki', 0)
        angle_Kd = rospy.get_param('~angle_Kd', 1)

        twist_Kp = rospy.get_param('~twist_Kp', 10)
        twist_Ki = rospy.get_param('~twist_Ki', 0)
        twist_Kd = rospy.get_param('~twist_Kd', 0.1)

        # set PID
        self.angle_PID = PID(angle_Kp, angle_Ki, angle_Kd, 100, 0, 'angle')
        self.rotate_PID = PID(rotate_Kp, rotate_Ki, rotate_Kd, 100, 0, 'rotate')
        self.twist_PID = PID(twist_Kp, twist_Ki, twist_Kd, 100, 0, 'twist')

        # publishers
        self.data_pub = rospy.Publisher('wrist_diff/data', Data)

        # subscribers
        self.des_ang_sub = rospy.Subscriber('joints/differential_lift/command', Float64, self.angle_cb)
        self.des_rot_sub = rospy.Subscriber('joints/differential_rotate/command', Float64, self.rotate_cb)
        self.des_twt_sub = rospy.Subscriber('joints/wrist_rotate/command', Float64, self.twist_cb)

        self.raw_pos_sub = rospy.Subscriber('wrist_diff/raw_pos', RawPosition, self.rawpos_cb)

        # initialize variables
        self.rotate_sp = 0
        self.angle_sp = 0
        self.twist_sp = 0

        self.angle = 0
        self.rotate = 0
        self.twist = 0

        self.recvddata = False
        self.shutting_down = False

        self.set_diff_left_spd = rospy.ServiceProxy(self.diff_left_name + '/set_speed', SetSpeed, persistent=True)
        self.set_diff_right_spd = rospy.ServiceProxy(self.diff_right_name + '/set_speed', SetSpeed, persistent=True)
        self.set_wrist_rotate_spd = rospy.ServiceProxy(self.wrist_rotate_name + '/set_speed', SetSpeed, persistent=True)

        rospy.on_shutdown(self.on_shutdown)

    def run(self):
        dt = 0.02
        self.rate = rospy.Rate(1/dt)

        while not rospy.is_shutdown():
            pass
            if not self.shutting_down:
                # set setpoints
                self.angle_PID.setSetpoint(self.angle_sp)
                self.rotate_PID.setSetpoint(self.rotate_sp)
                self.twist_PID.setSetpoint(self.twist_sp)

                # rospy.loginfo(rospy.get_name() + ': setpoints: %f %f' % (self.rotate_sp, self.angle_sp))
                if self.recvddata:
                    oldang = self.angle
                    oldrot = self.rotate
                    oldtwist = self.twist

                    self.twist = self.twist_input
                    self.angle = self.angle_input
                    self.rotate = self.rotate_input

                    # run PID control
                    self.angle_PID.update(self.angle, dt)
                    self.rotate_PID.update(self.rotate, dt)
                    self.twist_PID.update(self.twist, dt)

                    maxspd = 10
                    ang = self.angle_PID.getOutput()
                    rot = self.rotate_PID.getOutput()

                    twist_out = max(min(self.twist_PID.getOutput(), maxspd), -maxspd)
                    diff_left_out = max(min(ang + rot, maxspd), -maxspd)
                    diff_right_out = -max(min(ang - rot, maxspd), -maxspd)
                    rospy.loginfo('%.02f %.02f' % (diff_left_out, diff_right_out))

                    try:
                        self.set_diff_left_spd(diff_left_out)
                        self.set_diff_right_spd(diff_right_out)
                        self.set_wrist_rotate_spd(twist_out)
                    except rospy.ServiceException, e:
                        rospy.logwarn('Service did not process request: %s' % str(e))

                    # publish current data
                    self.data_pub.publish(timestamp=rospy.get_rostime(), 
                            angle=self.angle, 
                            rotate=self.rotate,
                            twist=self.twist,
                            anglespd=(self.angle-oldang)/dt,
                            rotatespd=(self.rotate-oldrot)/dt,
                            twistspd=(self.twist-oldtwist)/dt,
                            angle_output=self.angle_PID.output,
                            angle_error=self.angle_PID.error,
                            twist_output=self.twist_PID.output,
                            twist_error=self.twist_PID.error,
                            rotate_output=self.rotate_PID.output,
                            rotate_error=self.rotate_PID.error,
                            )
                else:
                    try:
                        self.set_diff_left_spd(0.0)
                        self.set_diff_right_spd(0.0)
                        self.set_wrist_rotate_spd(0.0)
                    except rospy.ServiceException, e:
                        rospy.logwarn('Service did not process request: %s' % str(e))

                self.rate.sleep()
            else: 
                try:
                    self.set_diff_left_spd(0.0)
                    self.set_diff_right_spd(0.0)
                    self.set_wrist_rotate_spd(0.0)
                except rospy.ServiceException, e:
                    rospy.logwarn('Service did not process request: %s' % str(e))

    def on_shutdown(self):
        try:
            self.set_diff_left_spd(0.0)
            self.set_diff_right_spd(0.0)
            self.set_wrist_rotate_spd(0.0)
        except rospy.ServiceException, e:
            rospy.logwarn('Service did not process request: %s' % str(e))

    def rotate_cb(self, data):
        pi = 3.1415926535
        self.rotate_sp = max(min(data.data, pi), -pi)

    def twist_cb(self, data):
        pi = 3.1415926535
        self.twist_sp = max(min(data.data, 2), -2)

    def angle_cb(self, data):
        pi = 3.1415926535
        self.angle_sp = max(min(data.data, pi/2), -pi/2)

    def rawpos_cb(self, data):
        pi = 3.1415926535
        self.angle_input = data.angle
        self.rotate_input = -data.rotate
        self.twist_input = -data.twist
        self.recvddata = True
        # rospy.loginfo(rospy.get_name() + ': rawpos: %.02f %.02f' % (self.angle, self.rotate))

if __name__ == '__main__':
    try:
        wrist = WristNode()    
        wrist.run()
    except rospy.ROSInterruptException:
        pass
