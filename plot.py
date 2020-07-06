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
bridge = CvBridge()

def translate_transform(p):
    t = np.eye(4)
    t[0,3] = p[0]
    t[1,3] = p[1]
    t[2,3] = p[2]
    return t

bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/align.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped','/car26/camera/color/camera_info','/car26/camera/color/image_throttled']
ps_24 = []
ps_26 = []
q_24 = None
q_26 = None
count = 0
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        q_24 = msg.pose.orientation
        ps_24 = msg.pose.position
    elif topic == '/car26/PoseStamped':
        q_26 = msg.pose.orientation
        ps_26 = msg.pose.position
    elif topic == '/car26/camera/color/image_throttled':
        count += 1
        if count == 1: continue

        q_temp = (q_24.x, q_24.y, q_24.z, q_24.w)
        pt24_T_w = transformerROS.fromTranslationRotation((ps_24.x, ps_24.y, ps_24.z), (q_24.x, q_24.y, q_24.z, q_24.w))
        fix24 = np.matrix([
            [ 1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        # pt24_T_w = np.matmul(pt24_T_w, fix24)

        x24_T_pt24 = translate_transform([0.5,0,0])
        x24_T_w = np.matmul(pt24_T_w, x24_T_pt24)
        plt.plot([ps_24.x, x24_T_w[0,3]], [ps_24.y, x24_T_w[1,3]], '.-r')

        y24_T_pt24 = translate_transform([0,0.5,0])
        y24_T_w = np.matmul(pt24_T_w, y24_T_pt24)
        plt.plot([ps_24.x, y24_T_w[0,3]], [ps_24.y, y24_T_w[1,3]], '.-g')
        print('24y (depth)', y24_T_w[2,3], ps_24.z)

        z24_T_pt24 = translate_transform([0,0,0.5])
        z24_T_w = np.matmul(pt24_T_w, z24_T_pt24)
        plt.plot([ps_24.x, z24_T_w[0,3]], [ps_24.y, z24_T_w[1,3]], '.-b')
        print('24z (depth)', z24_T_w[2,3], ps_24.z)

        pt26_T_w = transformerROS.fromTranslationRotation((ps_26.x, ps_26.y, ps_26.z), (q_26.x, q_26.y, q_26.z,q_26.w))
        # pt26_T_w = transformerROS.fromTranslationRotation((0,0,0), (q_26.z, q_26.x, q_26.y, -q_26.w))
        fix26 = np.matrix([
            [-1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        #pt26_T_w = np.matmul(pt26_T_w, fix26)
        

        x26_T_pt26 = translate_transform([0.5,0,0])
        x26_T_w = np.matmul(pt26_T_w, x26_T_pt26)
        plt.plot([ps_26.x, x26_T_w[0,3]], [ps_26.y, x26_T_w[1,3]], '.-m')

        y26_T_pt26 = translate_transform([0,0.5,0])
        y26_T_w = np.matmul(pt26_T_w, y26_T_pt26)
        plt.plot([ps_26.x, y26_T_w[0,3]], [ps_26.y, y26_T_w[1,3]], '.-y')

        z26_T_pt26 = translate_transform([0,0,0.5])
        z26_T_w = np.matmul(pt26_T_w, z26_T_pt26)
        plt.plot([ps_26.x, z26_T_w[0,3]], [ps_26.y, z26_T_w[1,3]], '.-c')
        print('26', z26_T_w[2,3], ps_26.z)



        # ps_26, q_26
        x = -ps_26.x
        y = ps_26.y
        matrix26 = tf.transformations.quaternion_matrix([q_26.x, q_26.y, q_26.z, q_26.w])

        xdir_x = x + matrix26[1,0]
        xdir_y = y + matrix26[0,0]
        # plt.plot([x, xdir_x], [y, xdir_y], '.-g')
        print(matrix26)


        print(count)
        plt.xlim([-3,3])
        plt.ylim([-3,3])
        plt.savefig('position' + str(count) + '.png')
        plt.clf()


        car26_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        plot_image = cv2.imread('/home/tudorf/mushr/position' + str(count) + '.png')
        combo = cv2.vconcat([car26_image, plot_image])
        cv2.imwrite('combo' + str(count) + '.png', combo)
