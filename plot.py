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
camModel = PinholeCameraModel()


def translate_transform(p):
    t = np.eye(4)
    t[0,3] = p[0]
    t[1,3] = p[1]
    t[2,3] = p[2]
    return t

def inverse_transform(t):
    R_inv = t[:3,:3].T
    p_inv = np.matmul(-R_inv,t[:3,3])
    t_inv = np.eye(4)
    t_inv[0,0] = R_inv[0,0]
    t_inv[0,1] = R_inv[0,1]
    t_inv[0,2] = R_inv[0,2]
    t_inv[1,0] = R_inv[1,0]
    t_inv[1,1] = R_inv[1,1]
    t_inv[1,2] = R_inv[1,2]
    t_inv[2,0] = R_inv[2,0]
    t_inv[2,1] = R_inv[2,1]
    t_inv[2,2] = R_inv[2,2]
    t_inv[0,3] = p_inv[0]
    t_inv[1,3] = p_inv[1]
    t_inv[2,3] = p_inv[2]
    return t_inv

bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/align.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped','/car26/camera/color/camera_info','/car26/camera/color/image_throttled']
ps_24 = None
ps_26 = None
q_24 = None
q_26 = None
camInfo = None
count = 0
imgs = []
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        q_24 = msg.pose.orientation
        ps_24 = msg.pose.position
    elif topic == '/car26/PoseStamped':
        q_26 = msg.pose.orientation
        ps_26 = msg.pose.position
    elif topic == '/car26/camera/color/camera_info':
        camInfo = msg
    elif topic == '/car26/camera/color/image_throttled':
        imgs.append(msg)
        count += 1
        if count < 2:
            continue

        q_temp = (q_24.x, q_24.y, q_24.z, q_24.w)
        pt24_T_w = transformerROS.fromTranslationRotation((ps_24.x, ps_24.y, ps_24.z), (q_24.x, q_24.y, q_24.z, q_24.w))
        fix24 = np.matrix([
            [ 1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        pt24_T_w = np.matmul(pt24_T_w, fix24)

        x24_T_pt24 = translate_transform([0.5,0,0])
        x24_T_w = np.matmul(pt24_T_w, x24_T_pt24)
        plt.plot([ps_24.x, x24_T_w[0,3]], [ps_24.y, x24_T_w[1,3]], '.-r')

        y24_T_pt24 = translate_transform([0,0.5,0])
        y24_T_w = np.matmul(pt24_T_w, y24_T_pt24)
        plt.plot([ps_24.x, y24_T_w[0,3]], [ps_24.y, y24_T_w[1,3]], '.-g')


        z24_T_pt24 = translate_transform([0,0,0.5])
        z24_T_w = np.matmul(pt24_T_w, z24_T_pt24)
        plt.plot([ps_24.x, z24_T_w[0,3]], [ps_24.y, z24_T_w[1,3]], '.-b')


        pt26_T_w = transformerROS.fromTranslationRotation((ps_26.x, ps_26.y, ps_26.z), (q_26.x, q_26.y, q_26.z,q_26.w))
        # pt26_T_w = transformerROS.fromTranslationRotation((0,0,0), (q_26.z, q_26.x, q_26.y, -q_26.w))
        fix26 = np.matrix([
            [ 1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 0, 1]
        ])
        pt26_T_w = np.matmul(pt26_T_w, fix26)
        

        x26_T_pt26 = translate_transform([0.5,0,0])
        x26_T_w = np.matmul(pt26_T_w, x26_T_pt26)
        plt.plot([ps_26.x, x26_T_w[0,3]], [ps_26.y, x26_T_w[1,3]], '.-m')

        y26_T_pt26 = translate_transform([0,0.5,0])
        y26_T_w = np.matmul(pt26_T_w, y26_T_pt26)
        plt.plot([ps_26.x, y26_T_w[0,3]], [ps_26.y, y26_T_w[1,3]], '.-y')

        z26_T_pt26 = translate_transform([0,0,0.5])
        z26_T_w = np.matmul(pt26_T_w, z26_T_pt26)
        plt.plot([ps_26.x, z26_T_w[0,3]], [ps_26.y, z26_T_w[1,3]], '.-c')


        w_T_pt26 = inverse_transform(pt26_T_w)
        pt24_T_pt26 = np.matmul(w_T_pt26, pt24_T_w)

        # the Camera Pinhole model uses +x right, +y down, +z forward
        pt_3d = (-pt24_T_pt26[1,3],-pt24_T_pt26[2,3],pt24_T_pt26[0,3])
        print('3d pt:', pt_3d)
        camModel.fromCameraInfo(camInfo)
        pt_2d = camModel.project3dToPixel(pt_3d)
        print('2d pt:', pt_2d)
        p2_round = (int(pt_2d[0]), int(pt_2d[1]))
        
        # x in 2d pt is y in 3d pt
        # y in 2d pt is z in 3d pt
        # focal_len = 0.0193
        # x_2d = pt_3d[1] / pt_3d[0] * focal_len * 640
        # y_2d = pt_3d[2] / pt_3d[0] * focal_len * 480
        # print(x_2d, y_2d)
        # p2_round = (int(x_2d + 320), int(y_2d + 240))
        # print(p2_round)



        car26_image = bridge.imgmsg_to_cv2(imgs[-2], "bgr8")
        cv2.circle(car26_image, p2_round, 10, (0,255,0), 3)

        


        print(count)
        plt.xlim([-3,3])
        plt.ylim([-3,3])
        plt.savefig('position' + str(count) + '.png')
        plt.clf()


        plot_image = cv2.imread('/home/tudorf/mushr/position' + str(count) + '.png')
        combo = cv2.hconcat([car26_image, plot_image])
        cv2.imwrite('combo' + str(count) + '.png', combo)
        