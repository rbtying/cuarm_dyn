#!/usr/bin/env python
import roslib
roslib.load_manifest('cuarm_dyn')
roslib.load_manifest('dynamixel_msgs')
import rospy
import serial
from std_msgs.msg import Header
from wrist_node.msg import RawPosition
from dynamixel_msgs.msg import JointState as DynState
from sensor_msgs.msg import JointState

class SensorNode:
    def __init__(self):
        rospy.init_node('sensor_node')

        # get parameters
        self.w3_addr = rospy.get_param('~w3_addr', 0)
        self.w1_addr = rospy.get_param('~w1_addr', 1)
        self.w2_addr = rospy.get_param('~w2_addr', 2)

        self.serialport = rospy.get_param('~sensor_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 1000000)

        # open serial port
        self.ser = serial.Serial(self.serialport, self.baud_rate, timeout=0.3)        

        # subscribers
        self.gripper_sub = rospy.Subscriber('hand_controller/state', DynState, self.gripper_cb)
        self.diff_left_sub = rospy.Subscriber('diff_left_controller/state', DynState, self.diff_left_cb)
        self.diff_right_sub = rospy.Subscriber('diff_right_controller/state', DynState, self.diff_right_cb)
        self.base_controller_sub = rospy.Subscriber('base_controller/state', DynState, self.base_cb)

        # publishers
        self.cur_pos_pub = rospy.Publisher('wrist_diff/raw_pos', RawPosition)
        self.wrist_rotate_pub = rospy.Publisher('joints/wrist_rotate/state', JointState)
        self.diff_lift_pub = rospy.Publisher('joints/differential_lift/state', JointState)
        self.diff_rotate_pub = rospy.Publisher('joints/differential_rotate/state', JointState)
        self.gripper_pub = rospy.Publisher('joints/gripper/state', JointState)

        self.left_diff_state = DynState()
        self.right_diff_state = DynState()
        self.base_control_state = DynState()

    def diff_left_cb(self, data):
        self.left_diff_state = data

    def diff_right_cb(self, data):
        self.right_diff_state = data

    def base_cb(self, data):
        self.base_control_state = data

    def gripper_cb(self, data):
        js = JointState()
        js.header = data.header
        js.name = ['Gripper']
        js.position = [data.current_pos]
        js.velocity = [data.velocity]
        self.gripper_pub.publish(js)

    def raw_to_radians(self, raw):
        pi = 3.1415926535
        return (raw * 1.0 - 2048) / 4096 * 2 * pi * 5

    def run(self):
        self.rate = rospy.Rate(50)
        buf = ''
        while not rospy.is_shutdown():
            self.ser.write('a')
            buf = buf + self.ser.read(self.ser.inWaiting())
            if '\r\n' in buf:
                lines = buf.split('\r\n')
                last_recv = lines[-2]
                vals = last_recv.split(',')

                try:
                    self.w1pos = 4096 - int(vals[self.w1_addr])
                    self.w2pos = int(vals[self.w2_addr])
                    self.w3pos = int(vals[self.w3_addr])

                    self.w1pos_rad = self.raw_to_radians(self.w1pos)
                    self.w2pos_rad = self.raw_to_radians(self.w2pos)
                    self.w3pos_rad = self.raw_to_radians(self.w3pos)

                    self.angle = (self.w1pos_rad + self.w2pos_rad)
                    self.rotate = -self.w1pos_rad + self.w2pos_rad

                    base_rotate_ratio = 1.0 / 5

                    self.cur_pos_pub.publish(timestamp=rospy.get_rostime(), 
                            w1pos=self.w1pos, 
                            w2pos=self.w2pos,
                            w3pos=self.w3pos,
                            w1pos_rad=self.w1pos_rad,
                            w2pos_rad=self.w2pos_rad,
                            w3pos_rad=self.w3pos_rad,
                            angle=self.angle,
                            rotate=self.rotate,
                            twist=self.w3pos_rad * base_rotate_ratio, 
                            )

                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = "0"

                    br_js = JointState()
                    br_js.header = header
                    br_js.name = ['base_rotate']
                    br_js.position = [twist]
                    self.wrist_rotate_pub(br_js)

                    dl_js = JointState()
                    dl_js.header = header
                    dl_js.name = ['differential_lift']
                    dl_js.position = [self.angle]
                    self.diff_lift_pub(dl_js)

                    dr_js = JointState()
                    dr_js.header = header
                    dr_js.name = ['differential_rotate']
                    dr_js.position = [self.rotate]
                    self.diff_rotate_pub(dr_js)
                except:
                    pass

                buf = lines[-1]
            self.rate.sleep()

if __name__ == '__main__':
    try:
        sensor = SensorNode()    
        sensor.run()
    except rospy.ROSInterruptException:
        pass
