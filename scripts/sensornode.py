#!/usr/bin/env python
import rospy
import serial
from wrist_node.msg import RawPosition

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

        # publishers
        self.cur_pos_pub = rospy.Publisher('wrist_diff/raw_pos', RawPosition)

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

                    self.angle = -(self.w1pos_rad + self.w2pos_rad)
                    self.rotate = -self.w1pos_rad + self.w2pos_rad
                    
                    self.cur_pos_pub.publish(timestamp=rospy.get_rostime(), 
                            w1pos=self.w1pos, 
                            w2pos=self.w2pos,
                            w3pos=self.w3pos,
                            w1pos_rad=self.w1pos_rad,
                            w2pos_rad=self.w2pos_rad,
                            w3pos_rad=self.w3pos_rad,
                            angle=self.angle,
                            rotate=self.rotate,
                            twist=self.w3pos_rad / 5, 
                            )
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
