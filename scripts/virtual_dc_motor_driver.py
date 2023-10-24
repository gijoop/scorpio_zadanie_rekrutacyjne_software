#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16, Float32

class MotorDriver:
    def __init__(self):
        self.prev_time = rospy.get_rostime()
        self.prev_pos = None

        self.pub = rospy.Publisher("/virtual_dc_motor_driver/get_velocity", Float32, queue_size=10)
        self.sub = rospy.Subscriber("/virtual_dc_motor/get_position", UInt16, callback=self.driver_callback)

    def driver_callback(self, pos: UInt16):
        time = rospy.get_rostime()
        time_diff = time - self.prev_time

        if not self.prev_pos:
            self.prev_pos = pos
        pos_diff = self.calculateDifference(self.prev_pos.data, pos.data, 4096)
        RPM_ = (pos_diff/4096) / (float(time_diff.nsecs) / (60 * 1000000000)) #RPM = evolutions/minute
        RPM = Float32(RPM_)
        self.prev_time = time
        self.prev_pos = pos
        self.pub.publish(RPM)

    def calculateDifference(self, a, b, base):
        return int(base/2 - abs(base/2 - ((a-b)%base)))

if __name__=="__main__":
    rospy.init_node("virtual_dc_motor_driver")
    rospy.loginfo("Node has been started")

    MotorDriver()
    rospy.spin()
