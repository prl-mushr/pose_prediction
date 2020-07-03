import rosbag
import numpy as np

import tf
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2

import matplotlib.pyplot as plt

transformerROS = tf.TransformerROS()
def translate_transform(p):
    t = np.eye(4)
    t[0,3] = p[0]
    t[1,3] = p[1]
    t[2,3] = p[2]
    return t

bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/output.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped','/car24/camera/color/camera_info','/car24/camera/color/image_throttled']
ps_24 = []
ps_26 = []
xs_24 = []
ys_24 = []
xs_26 = []
ys_26 = []
q_24 = None
q_26 = None
count = 0
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        xs_24.append(msg.pose.position.x)
        ys_24.append(msg.pose.position.y)
        q_24 = msg.pose.orientation
        ps_24.append(msg.pose.position)
    elif topic == '/car26/PoseStamped':
        xs_26.append(-msg.pose.position.x)
        ys_26.append(msg.pose.position.y)
        q_26 = msg.pose.orientation
        ps_26.append(msg.pose.position)
    elif topic == '/car24/camera/color/image_throttled':
        count += 1
        if count == 1: continue
        q_temp = (q_24.x, q_24.y, q_24.z, q_24.w)
        pt24_T_w = transformerROS.fromTranslationRotation((ps_24[-1].x, ps_24[-1].y, ps_24[-1].z), (-q_24.z, q_24.x, -q_24.y, -q_24.w))
        fix24 = np.matrix([
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [-1, 0, 0, 0],
            [ 0, 0, 0, 1]
        ])
        pt24_T_w = np.matmul(pt24_T_w, fix24)

        x24_T_pt24 = translate_transform([0.5,0,0])
        x24_T_w = np.matmul(pt24_T_w, x24_T_pt24)
        plt.plot([xs_24[-1], x24_T_w[0,3]], [ys_24[-1], x24_T_w[1,3]], '.-g')

        y24_T_pt24 = translate_transform([0,0.5,0])
        y24_T_w = np.matmul(pt24_T_w, y24_T_pt24)
        plt.plot([xs_24[-1], y24_T_w[0,3]], [ys_24[-1], y24_T_w[1,3]], '.-m')
        print('24y', y24_T_w[2,3], ps_24[-1].z)

        z24_T_pt24 = translate_transform([0,0,0.5])
        z24_T_w = np.matmul(pt24_T_w, z24_T_pt24)
        plt.plot([xs_24[-1], z24_T_w[0,3]], [ys_24[-1], z24_T_w[1,3]], '.-c')
        print('24z', z24_T_w[2,3], ps_24[-1].z)

        pt26_T_w = transformerROS.fromTranslationRotation((-ps_26[-1].x, ps_26[-1].y, ps_26[-1].z), (-q_26.x, q_26.y, -q_26.z, q_26.w))
        fix26 = np.matrix([
            [-1, 0, 0, 0],
            [ 0, 0,-1, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        pt26_T_w = np.matmul(pt26_T_w, fix26)
        

        x26_T_pt26 = translate_transform([0.5,0,0])
        x26_T_w = np.matmul(pt26_T_w, x26_T_pt26)
        plt.plot([xs_26[-1], x26_T_w[0,3]], [ys_26[-1], x26_T_w[1,3]], '.-g')

        y26_T_pt26 = translate_transform([0,0.5,0])
        y26_T_w = np.matmul(pt26_T_w, y26_T_pt26)
        plt.plot([xs_26[-1], y26_T_w[0,3]], [ys_26[-1], y26_T_w[1,3]], '.-m')

        z26_T_pt26 = translate_transform([0,0,0.5])
        z26_T_w = np.matmul(pt26_T_w, z26_T_pt26)
        plt.plot([xs_26[-1], z26_T_w[0,3]], [ys_26[-1], z26_T_w[1,3]], '.-c')
        print('26', z26_T_w[2,3], ps_26[-1].z)


        print(count)
        # plt.plot(xs_24, ys_24, '.-r')
        # plt.plot(xs_26, ys_26, '.-b')
        plt.xlim([-3,3])
        plt.ylim([-3,3])
        plt.savefig('position' + str(count) + '.png')
        #plt.clf()


