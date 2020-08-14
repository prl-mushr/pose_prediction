# Trim the original bag file to specified range

import rosbag
import rospy
import numpy as np

import tf
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import cv2


# Modify these
read_bag = rosbag.Bag('/home/ugrads/hard_data/carpose/bags/car37_mocap.bag')
write_bag = rosbag.Bag('/home/ugrads/hard_data/carpose/bags/car37_trim.bag', 'w')
BEGIN_TIME = 60
END_TIME = 12000
topics = ['/vrpn_client_node/car37/pose','/vrpn_client_node/car38/pose','/car37/camera/color/camera_info','/car37/camera/color/image_throttled', '/vrpn_client_node/car35/pose']


start_time = None
flag = True
for topic, msg, t in read_bag.read_messages(topics=topics):
    if start_time is None:
        start_time = t
    if t - start_time < rospy.Duration(secs=BEGIN_TIME):
        continue
    if t - start_time > rospy.Duration(secs=END_TIME):
        print('closing')
        break
    if flag:
        print('skipped')
        flag = False
    write_bag.write(topic, msg, t=t)

read_bag.close()
write_bag.close()