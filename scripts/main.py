#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import *

class Maze():
    def __init__(self):
        print("Start Maze Tool for Raspberry Pi Mouse Sim.") 
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        self.data = LightSensorValues()

    def sensor_callback(self, msg):
        self.data = msg

    def motor_cont(self, left_hz, right_hz):
        if not rospy.is_shutdown():
            d = MotorFreqs()
            d.left_hz = left_hz
            d.right_hz = right_hz
            self.motor_raw_pub.publish(d)

    def calc(self, ls, rs, lf, vel):
        offset = 100
        if rs > 3000:
            self.motor_cont(-offset, offset)
            return
        if ls > 3000:
            self.motor_cont(offset, -offset)
            return

        diff = (ls - 1600) * 0.05
        self.motor_cont(offset + diff, offset - diff)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.calc(self.data.left_side, self.data.right_side, self.data.left_forward, 100)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Clustering_Server')
    Maze().run()
