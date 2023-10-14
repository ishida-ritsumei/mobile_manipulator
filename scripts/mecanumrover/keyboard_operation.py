#! /usr/bin/env python3
# -*- coding: utf-8 -*- # 日本語を使うためのおまじない。#これがないとエラーが出る
#キーボードで速度入力（全て手動操作

import rospy
from geometry_msgs.msg import Twist
from math import sin, cos, pi
import numpy as np
import sys



class operation:


    def __init__(self): #初期位置・角度・速度・角速度
        rospy.init_node('range', anonymous=True)
        self.vel_pub = rospy.Publisher('/rover_twist', Twist, queue_size=50)
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.vx = 0
        self.vy = 0
        self.vtheta = 0
        self.x = 0 
        self.y = 0
        self.theta = 0


    def set_vel(self, vx, vy, vtheta):

        vel = Twist()
        vel.linear.x  = vx # 並進速度vx [m/s]
        vel.linear.y  = vy # 並進速度vy[m/s]
        vel.linear.z  = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = vtheta  # 角速度 [rad/s]
        self.vel_pub.publish(vel)#これがないと動かない

    
    def main(self):

        print("Let's move your robot")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rospy.loginfo("x=%s,vx=%s,y=%s,vy=%s,theta=%s,vtheta=%s",self.x,self.vx,self.y,self.vy,self.theta,self.vtheta)
            
            self.last_time = rospy.Time.now()
            
            #キーボード入力
            key = ord(input("Keyboard Input:"))

            #現在の位置・姿勢の計算
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            delta_theta = self.vtheta * dt
            self.theta += delta_theta
            delta_x = (self.vx * cos(self.theta) - self.vy * sin(self.theta)) * dt
            delta_y = (self.vx * sin(self.theta) + self.vy * cos(self.theta)) * dt
            self.x += delta_x
            self.y 	+= delta_y
            

            if key == ord('w'): #前
                self.vx = self.vx + 0.1
                self.vy = self.vy + 0
                self.vtheta = self.vtheta + 0
            if key == ord('x'): #後ろ
                self.vx = self.vx - 0.1              
            if key == ord('a'): #右
                self.vy = self.vy + 0.1              
            if key == ord('d'): #左
                self.vy = self.vy - 0.1              
            if key == ord('c'): #右回転
                self.vtheta = self.vtheta - pi/6              
            if key == ord('z'): #左回転
                self.vtheta = self.vtheta + pi/6               
            if key == ord('s'): #止まる
                self.vx = 0
                self.vy = 0
                self.vtheta = 0
                
            if key == 27 or key == ord('q'): #処理終了
                self.set_vel(0, 0, 0)
                print("処理終了")
                sys.exit()

            
            self.set_vel(self.vx, self.vy, self.vtheta)
            
            rate.sleep()
            
                        


if __name__ == '__main__':
    try:
        robot = operation()
        robot.main()
        rospy.spin()

    except rospy.ROSInterruptException: pass
