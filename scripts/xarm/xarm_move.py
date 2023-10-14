#!/usr/bin/env python3

import rospy
from xarm_msgs.srv import Move

rospy.wait_for_service('/xarm/move_joint')

payload = Move
payload.pose     = [0.0,0,0,0,0,0]
payload.mvvelo   = 0.35
payload.mvacc    = 7
payload.mvtime   = 0
payload.mvradii  = 0

response = rospy.ServiceProxy('/xarm/move_joint', Move)

res = response(payload.pose, payload.mvvelo, payload.mvacc, payload.mvtime, payload.mvradii)


payload.pose = [0.1, 0, 0,0,0,0]
res = response(payload.pose, payload.mvvelo, payload.mvacc, payload.mvtime, payload.mvradii)

payload.pose[1] = 0.1
res = response(payload.pose, payload.mvvelo, payload.mvacc, payload.mvtime, payload.mvradii)

print(res)


# from xarm.wrapper import XArmAPI

# #初期設定
# arm = XArmAPI('192.168.1.213')
# arm.motion_enable(enable=True)
# arm.set_gripper_enable(enable=True)
# arm.set_mode(0)
# arm.set_state(state=0)

# #動きを指定
# arm.reset(wait=True)
# arm.set_gripper_position(800, wait=True)
# arm.set_position(x=400, y=0, z=200, roll=180, pitch=0, yaw=0, speed=100, wait=True)
# arm.set_position(x=400, y=0, z=0, roll=180, pitch=0, yaw=0, speed=100, wait=True)
# arm.set_gripper_position(0, wait=True)

# #切断
# arm.disconnect()
