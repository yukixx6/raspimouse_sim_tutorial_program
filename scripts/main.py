#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import *

class Maze():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # モータに周波数を入力するためのパブリッシャー
        self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        # Raspberry Pi Mouseの光センサのメッセージオブジェクト
        self.data = LightSensorValues()

        self.ls_count = 0
        self.rs_count = 0

    def sensor_callback(self, msg):
        # クラス変数のメッセージオブジェクトに受信したデータをセット
        self.data = msg

    def motor_cont(self, left_hz, right_hz):
        if not rospy.is_shutdown():
            d = MotorFreqs()
            # 両輪の周波数を設定
            d.left_hz = left_hz
            d.right_hz = right_hz
            # パブリッシュ
            self.motor_raw_pub.publish(d)

    def turn_move(self, m):
        if m == "LEFT": self.motor_cont(-200, 200)
        if m == "RIGHT": self.motor_cont(200, -200)

    def moveFeedback(self, offset, speed, k):
        # left_sideが2000より大きい時は、右回り旋回
        if self.data.left_side > 1500:
            self.turn_move("RIGHT")
            return
        
        # right_sideが2000より大きい時は、右回り旋回
        if self.data.right_side > 1500:
            self.turn_move("LEFT")
            return

        # 壁沿いを追従走行するための計算
        # (基準値 - 現在のleft_side) * ゲイン
        diff = (offset - self.data.left_side) * k
        # 計算した値をモータに出力
        self.motor_cont(speed - diff, speed + diff)

    def stopMove(self):
        # 終了時にモータを止める
        self.motor_cont(0, 0)

    def motion(self):
        # 左側に壁があるとき
        if self.data.left_side > self.data.right_side: self.rs_count += 1
        else: self.ls_count += 1

        # 袋小路の処理
        if self.ls_count < self.rs_count: self.state_turn = "RIGHT"
        else: self.state_turn = "LEFT"
        if self.data.left_forward > 1000 and self.data.right_forward > 1000:
            while self.data.left_forward > 500 and self.data.right_forward > 500:
                self.turn_move(self.state_turn)
                self.rate.sleep()
            self.ls_count = 0
            self.rs_count = 0
            return
        self.moveFeedback(600, 500, 0.2)

    def run(self):
        self.rate = rospy.Rate(50)
        self.state_wall = False
        rospy.on_shutdown(self.stopMove)
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Maze')
    Maze().run()
