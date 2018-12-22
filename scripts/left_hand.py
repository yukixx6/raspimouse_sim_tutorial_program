#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty

class LeftHand():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # モータに周波数を入力するためのパブリッシャー
        self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        # Raspberry Pi Mouseの光センサのメッセージオブジェクト
        self.data = LightSensorValues()

        # 実行時にシミュレータを初期状態にする
        self.modeSimReset = True

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

    def moveFeedback(self, offset, speed, k, mode):
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
        if mode == "LEFT":
            diff = (offset - self.data.left_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed - diff, speed + diff)
        if mode == "RIGHT":
            diff = (offset - self.data.right_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed + diff, speed - diff)

    def stopMove(self):
        # 終了時にモータを止める
        self.motor_cont(0, 0)

    def checker(self):
        # 壁無し判定
        if self.data.left_side < 100:
            print("--RS_COUNT:", self.data.left_side)
            self.rs_count += 1
        if self.data.right_side < 150:
            print("--LS_COUNT:", self.data.right_side)
            self.ls_count += 1

    def motion(self):
        # 左側に壁がある確率が高くて、目の前に壁がなさそうなとき
        if self.data.left_forward < 300 or self.data.right_forward < 300:
            print("Move: STRAIGHT")
            for time in range(12):
                self.checker()
                if self.data.left_side > self.data.right_side:
                    self.moveFeedback(500, 500, 0.2, "LEFT")
                else:
                    self.moveFeedback(500, 500, 0.2, "RIGHT")
                self.rate.sleep()
            self.stopMove()
            
            # 目の前に壁がなくて、右側に壁がない場合
            if self.data.left_forward < 300 or self.data.right_forward < 300:
                if self.rs_count > 0:
                    print("Move: MID LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
            # 直進した後に、目の前に壁があったとき
            elif self.data.left_forward > 300 and self.data.right_forward > 300:
                # 左右の壁がない場合
                if self.ls_count > 0 and self.rs_count > 0:
                    print("Move: LEFT TURN_2")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
                # 右の壁がない場合
                elif self.ls_count > 0:
                    print("Move: RIGHT TURN")
                    for time in range(10):
                        self.turn_move("RIGHT")
                        self.rate.sleep()
                    self.stopMove()
                # 左の壁がない場合
                elif self.rs_count > 0:
                    print("Move: LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()          
            self.ls_count = 0
            self.rs_count = 0
            return
        
        # 左右関係なく、目の前に壁があるとき
        if self.data.left_forward > 2000 and self.data.right_forward > 2000:
            print("Move: DEAD END")
            for time in range(20):
                self.turn_move("LEFT")
                self.rate.sleep()
            self.stopMove()
            self.ls_count = 0
            self.rs_count = 0
            return
        if self.data.left_side > self.data.right_side:
            self.moveFeedback(500, 500, 0.2, "LEFT")
        else:
            self.moveFeedback(500, 500, 0.2, "RIGHT")

    def init(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException, e: print "Service call failed: %s"%e
        rospy.wait_for_service('/motor_on')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException, e: print "Service call failed: %s"%e
        
    def run(self):
        self.rate = rospy.Rate(10)
        self.init()
        rospy.on_shutdown(self.stopMove)
        while self.data.left_side == 0 and self.data.right_side == 0:
            self.rate.sleep()
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('LeftHand')
    LeftHand().run()
