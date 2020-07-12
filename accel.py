# Plots the acceleration from the original car bag file for comparison to the mocap data

import rosbag
import rospy
import numpy as np

import tf
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2

import matplotlib.pyplot as plt

bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/car24_trim.bag')

start_time = None
ts = []
xs = []
ys = []
zs = []
maxz = -10000
maxzt = 0
for topic, msg, t in bag.read_messages(topics=['/camera/accel/sample']):
    if start_time is None: start_time = t
    if t - start_time < rospy.Duration(16.5): continue
    if t - start_time > rospy.Duration(18.5): break
    ts.append(t.to_sec())
    # xs.append(msg.linear_acceleration.x)
    maxz = max(maxz, msg.linear_acceleration.z)
    if msg.linear_acceleration.z == maxz: maxzt = t
    # ys.append(msg.linear_acceleration.y)
    zs.append(msg.linear_acceleration.z)

print(start_time)
print(maxz, maxzt - start_time)
# plt.plot(ts, xs, '.-r')
# plt.plot(ts, ys, '.-g')
plt.plot(ts, zs, '.-b')
plt.savefig('accel.png')
