#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8, Int16, Float32

class MotorController:
    def __init__(self):
        self.velocity_goal = 0.0
        self.power = 0
        self.pub = rospy.Publisher("/virtual_dc_motor/set_cs", Int8, queue_size=10)
        rospy.Subscriber("/virtual_dc_motor_controller/set_velocity_goal", Int16, callback=self.velocity_goal_callback)
        rospy.Subscriber("/virtual_dc_motor_driver/get_velocity", Float32, callback=self.controller_callback)
        
    def velocity_goal_callback(self, vel_goal: Int8):
        self.velocity_goal = vel_goal.data

    def controller_callback(self, vel: Float32):
        velocity = vel.data
        if velocity < self.velocity_goal and abs(self.power) < 100:
            self.power += 1
        elif velocity == self.velocity_goal:
            self.power += 0
        elif velocity > self.velocity_goal and abs(self.power) > 0:
            self.power -= 1
        self.pub.publish(self.power)

if __name__=="__main__":
    rospy.init_node("virtual_dc_motor_controller")
    rospy.loginfo("Node has been started")

    MotorController()
    rospy.spin()