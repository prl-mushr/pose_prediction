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

read_bag = rosbag.Bag('/home/tudorf/mushr/car37_mocap.bag')

write_bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/car37_trim.bag', 'w')

start_time = None
for topic, msg, t in read_bag.read_messages(['/vrpn_client_node/car37/pose','/vrpn_client_node/car38/pose','/car37/camera/color/camera_info','/car37/camera/color/image_throttled', '/vrpn_client_node/car35/pose']):
    if start_time is None:
        start_time = t
    if t - start_time < rospy.Duration(secs=150):
        continue
    if t - start_time > rospy.Duration(secs=180):
        break
    write_bag.write(topic, msg, t=t)

read_bag.close()
write_bag.close()