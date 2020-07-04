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

read_bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/output.bag')

write_bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/align.bag', 'w')

start_time = None
for topic, msg, t in read_bag.read_messages():
    if start_time is None: start_time = t
    if topic[:6] == '/car24':
        corrected_time = t - rospy.Duration.from_sec(7.1833)
        if corrected_time >= start_time:
            write_bag.write(topic, msg, t=corrected_time)
    elif topic[:6] == '/car26':
        if t - start_time < rospy.Duration.from_sec(18.7):
            write_bag.write(topic, msg, t=t)       

write_bag.close()
read_bag.close()