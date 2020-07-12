import rosbag
import numpy as np

import tf
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2

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

def translate_transform(p):
    t = np.eye(4)
    t[0,3] = p[0]
    t[1,3] = p[1]
    t[2,3] = p[2]
    return t

def roll90(t):
    fix26 = np.matrix([
            [ 1, 0, 0, 0],
            [ 0, 0,-1, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 0, 1]
        ])
    return np.matmul(t, fix26)

# 1 2 3
# 4 5 6
# 7 8 9




transformerROS = tf.TransformerROS()
bridge = CvBridge()
pinhole = PinholeCameraModel()

##
trackedPt_T_baselink = translate_transform([-0.058325, 0, 0.08125])
# trackedPt_T_baselink = transformerROS.fromTranslationRotation((-0.058325, 0, 0.08125), (-0.5, 0.5, -0.5, 0.5))
baselink_T_trackedPt = inverse_transform(trackedPt_T_baselink)

# colorCam_T_baselink = transformerROS.fromTranslationRotation((0.012, 0.033, 0.068), (-0.5, 0.5, -0.5, 0.5))
colorCam_T_baselink = translate_transform([0.012, 0.033, 0.068])

base_link_back  = -0.2
base_link_front =  0.1
base_link_left  = -0.125
base_link_right =  0.125
base_link_down  = -0.075
base_link_up    =  0.1

bld_T_baselink = translate_transform([base_link_back, base_link_left, base_link_down])
blu_T_baselink = translate_transform([base_link_back, base_link_left, base_link_up])
brd_T_baselink = translate_transform([base_link_back, base_link_right, base_link_down])
bru_T_baselink = translate_transform([base_link_back, base_link_right, base_link_up])
fld_T_baselink = translate_transform([base_link_front, base_link_left, base_link_down])
flu_T_baselink = translate_transform([base_link_front, base_link_left, base_link_up])
frd_T_baselink = translate_transform([base_link_front, base_link_right, base_link_down])
fru_T_baselink = translate_transform([base_link_front, base_link_right, base_link_up])

corner_transforms = [bld_T_baselink, blu_T_baselink, brd_T_baselink, bru_T_baselink,
                     fld_T_baselink, flu_T_baselink, frd_T_baselink, fru_T_baselink]

##


bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/align.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped','/car26/camera/color/camera_info','/car26/camera/color/image_throttled']
car24PoseStamps = []
car26PoseStamps = []
cameraInfos = []
images = []
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        car24PoseStamps.append((msg, t))
    elif topic == '/car26/PoseStamped':
        car26PoseStamps.append((msg, t))
    elif topic == '/car26/camera/color/camera_info':
        cameraInfos.append((msg, t))
    elif topic == '/car26/camera/color/image_throttled':
        images.append((msg, t))

idx24 = 0
idx26 = 0
idxInfo = 0
for idxImage, (msg, t) in enumerate(images):
    if idxImage > 60: break
    # bring rest of data into sync
    while idx24 < len(car24PoseStamps) - 1 and t > car24PoseStamps[idx24][1]:
        idx24 += 1
    while idx26 < len(car26PoseStamps) - 1 and t > car26PoseStamps[idx26][1]:
        idx26 += 1
    while idxInfo < len(cameraInfos) - 1 and t > cameraInfos[idxInfo][1]:
        idxInfo += 1
    

    car24PoseStamp = car24PoseStamps[idx24][0]
    car26PoseStamp = car26PoseStamps[idx26][0]

    car24Pose = car24PoseStamp.pose
    car24_T_w = transformerROS.fromTranslationRotation((car24Pose.position.x, car24Pose.position.y, car24Pose.position.z),
        (car24Pose.orientation.x, car24Pose.orientation.y, car24Pose.orientation.z, car24Pose.orientation.w))
    car24_T_w = roll90(car24_T_w)
    
    car26Pose = car26PoseStamp.pose
    car26_T_w = transformerROS.fromTranslationRotation((-car26Pose.position.x, car26Pose.position.y, car26Pose.position.z),
        (car26Pose.orientation.x, car26Pose.orientation.y, car26Pose.orientation.z, car26Pose.orientation.w))
    car26_T_w = roll90(car26_T_w)

    baselink24_T_w = np.matmul(car24_T_w, baselink_T_trackedPt)
    baselink26_T_w = np.matmul(car26_T_w, baselink_T_trackedPt)

    cam26_T_w = np.matmul(baselink26_T_w, colorCam_T_baselink)

    camInfoMsg = cameraInfos[idxInfo][0]
    imageMsg = msg
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    pinhole.fromCameraInfo(camInfoMsg)

    # w_T_car24 = inverse_transform(car24_T_w)
    # car24_T_cam26 = np.matmul(w_T_car24, cam26_T_w)
    # relative_pt = (car24_T_cam26[0,3], car24_T_cam26[1,3], car24_T_cam26[2,3])
    # p2 = pinhole.project3dToPixel(relative_pt)
    # print(p2)
    # p2_round = (int(p2[0]), int(p2[1]))
    # cv2.circle(cv_image, p2_round, 10, (255,255,255), 3)

    # for corner_n, corner_transform in enumerate(corner_transforms):
    #     corner24_T_w = np.matmul(baselink26_T_w, corner_transform)
    #     w_T_corner24 = inverse_transform(corner24_T_w)
    #     corner24_T_cam26 = np.matmul(w_T_corner24, cam26_T_w)

    #     relative_pt = (corner24_T_cam26[0,3], corner24_T_cam26[1,3], corner24_T_cam26[2,3])
    #     p2 = pinhole.project3dToPixel(relative_pt)
    #     print(p2)
    #     p2_round = (int(p2[0]), int(p2[1]))
    #     if corner_n < 4:
    #         cv2.circle(cv_image, p2_round, 5, (255,255,0), 1)
    #     else:
    #         cv2.circle(cv_image, p2_round, 5, (0,255,255), 1)

    
    w_T_car26 = inverse_transform(car26_T_w)
    car24_T_car26 = np.matmul(w_T_car26, car24_T_w)
    # the Camera Pinhole model uses +x right, +y down, +z forward
    
    relative_pt = (-car24_T_car26[1,3], -car24_T_car26[2,3], car24_T_car26[0,3])
    p2 = pinhole.project3dToPixel(relative_pt)
    print(p2)
    p2_round = (int(p2[0]), int(p2[1]))
    cv2.circle(cv_image, p2_round, 10, (0,255,0), 3)

    cv2.imwrite("/home/tudorf/mushr/image" + str(idxImage) + ".png", cv_image)
    