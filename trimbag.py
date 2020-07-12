# Trim the original bag file to first 30 seconds

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

read_bag = rosbag.Bag('/home/tudorf/mushr/Car24.bag')

write_bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/car24_trim.bag', 'w')

start_time = None
for topic, msg, t in read_bag.read_messages():
    if start_time is None:
        start_time = t
    if t - start_time > rospy.Duration(secs=30):
        break
    write_bag.write(topic, msg, t=t)

read_bag.close()
write_bag.close()