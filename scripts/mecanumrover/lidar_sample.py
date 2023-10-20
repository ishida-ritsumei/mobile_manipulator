#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np



class lidar_data:


    def __init__(self):
        rospy.init_node('range', anonymous=True)
        self.odom_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher('/rover_twist', Twist, queue_size=10)


    def set_vel(self, vx, vy):

        vel = Twist()
        vel.linear.x  = vx  # 並進速度
        vel.linear.y  = vy
        vel.linear.z  = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0  # 角速度

        self.vel_pub.publish(vel)

        #rospy.loginfo("Velocity: Linear=%s Angular=%s", vel.linear.x, vel.angular.z)#表示



    def lidar_callback(self, msg):


        self.ranges = msg.ranges[0:1793]
        self.left = self.ranges[0:897]
        self.right = self.ranges[898:1793]
        self.count_right = self.right.count(float("inf"))
        self.count_left = self.left.count(float("inf"))
        self.scope = self.ranges[718:1076]
        self.less = [i for i in self.scope if i < 1.0]

        A = 3
        self.ranges_2 = msg.ranges[1794:3584] * A
        self.left_2 = self.ranges_2[1794:2691]
        self.right_2 = self.ranges_2[2692:3584]
        self.count_right_2 = self.right_2.count(float("inf"))
        self.count_left_2 = self.left_2.count(float("inf"))
        self.scope_2 = self.ranges_2[718:1076]
        self.less_2 = [i for i in self.scope_2 if i < 1.0]


    def main(self):

        print("Let's move your robot")
        x_vel  = input ("Input x velocity [m/s] :")
        y_vel = input ("Input y velocity [m/s]:")
        self.set_vel(x_vel, y_vel)
    
        while not rospy.is_shutdown():

            print(any(self.less))
            print(any(self.less_2))
            print("\n")
        
            if any(self.less) == True:
                if self.count_right < self.count_left:
                    self.set_vel(0, 0.5)
                    rospy.loginfo("left=%s right=%s", self.count_left, self.count_right)

                else:
                    self.set_vel(0,(-0.5))
                    rospy.loginfo("left=%s right=%s", self.count_left, self.count_right)

            elif any(self.less_2) == True:
                if self.count_right_2 < self.count_left_2:
                    self.set_vel(0, 0.5)
                    rospy.loginfo("left=%s right=%s", self.count_left_2, self.count_right_2)

                else:
                    self.set_vel(0,(-0.5))
                    rospy.loginfo("left=%s right=%s", self.count_left_2, self.count_right_2)

            else :
                self.set_vel(x_vel, y_vel)
                   

if __name__ == '__main__':
    try:
        robot = lidar_data()
        robot.main()
        rospy.spin()

    except rospy.ROSInterruptException: pass
