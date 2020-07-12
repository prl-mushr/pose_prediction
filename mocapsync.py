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

def fixRotation(pt24_T_w):
    fix = np.matrix([
            [ 1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 0, 1]
    ])
    return np.matmul(pt24_T_w, fix)

trackedPt_T_baselink = translate_transform([-0.058325, 0, 0.08125])
baselink_T_trackedPt = inverse_transform(trackedPt_T_baselink)
colorCam_T_baselink = translate_transform([0.012, 0.033, 0.068])

bag = rosbag.Bag('/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/align.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped','/car26/camera/color/camera_info','/car26/camera/color/image_throttled']

ps_24 = []
ps_26 = []
camInfo_26 = []
cam_26 = []

for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        ps_24.append((t, msg.pose))
    elif topic == '/car26/PoseStamped':
        ps_26.append((t, msg.pose))
    elif topic == '/car26/camera/color/camera_info':
        camInfo_26.append((t, msg))
    elif topic == '/car26/camera/color/image_throttled':
        cam_26.append((t, msg))

idx26 = 0
idxInfo = 0
idxImg = 0
for idx24 in range(len(ps_24)):
    targetT = ps_24[idx24][0]
    while idx26 < len(ps_26)-1 and ps_26[idx26][0] < targetT: idx26 += 1
    while idxInfo < len(camInfo_26)-1 and camInfo_26[idxInfo][0] < targetT: idxInfo += 1
    while idxImg < len(cam_26)-1 and cam_26[idxImg][0] < targetT: idxImg += 1

    ps24 = ps_24[idx24][1]
    ps26 = ps_26[idx26][1]
    camInfo26 = camInfo_26[idxInfo][1]
    img26 = cam_26[idxImg+1][1]
    
    pos24 = ps24.position
    ori24 = ps24.orientation
    pos26 = ps26.position
    ori26 = ps26.orientation

    pt24_T_w = transformerROS.fromTranslationRotation((pos24.x, pos24.y, pos24.z), (ori24.x, ori24.y, ori24.z, ori24.w))
    pt24_T_w = fixRotation(pt24_T_w)
    pt26_T_w = transformerROS.fromTranslationRotation((pos26.x, pos26.y, pos26.z), (ori26.x, ori26.y, ori26.z, ori26.w))
    pt26_T_w = fixRotation(pt26_T_w)

    base24_T_w = np.matmul(pt24_T_w, baselink_T_trackedPt)
    cam24_T_w = np.matmul(base24_T_w, colorCam_T_baselink)
    base26_T_w = np.matmul(pt26_T_w, baselink_T_trackedPt)
    cam26_T_w = np.matmul(base26_T_w, colorCam_T_baselink)

    w_T_cam26 = inverse_transform(cam26_T_w)
    cam24_T_cam26 = np.matmul(w_T_cam26, cam24_T_w)

    # the Camera Pinhole model uses +x right, +y down, +z forward
    pt_3d = (-cam24_T_cam26[1,3],-cam24_T_cam26[2,3],cam24_T_cam26[0,3])
    camModel.fromCameraInfo(camInfo26)
    pt_2d = camModel.project3dToPixel(pt_3d)
    p2_round = (int(pt_2d[0]), int(pt_2d[1]))
    
    car26_image = bridge.imgmsg_to_cv2(img26, "bgr8")
    cv2.circle(car26_image, p2_round, 10, (0,255,0), 3)

    combo = car26_image
    if idxImg - 4 >= 0:
        img26_1 = cam_26[idxImg-3][1]
        img26_2 = cam_26[idxImg-4][1]
        car26_image_1 = bridge.imgmsg_to_cv2(img26_1, "bgr8")
        cv2.circle(car26_image_1, p2_round, 10, (0,255,0), 3)
        car26_image_2 = bridge.imgmsg_to_cv2(img26_2, "bgr8")
        cv2.circle(car26_image_2, p2_round, 10, (0,255,0), 3)
        combo = cv2.hconcat([car26_image, car26_image_1])
        combo = cv2.hconcat([combo, car26_image_2])

    cv2.imwrite('mocap24_new_' + str(idx24) + '_' + str(idxImg) + '.png', combo)
    if idx24 == 240: break













# idx24 = 0
# idx26 = 0
# idxInfo = 0
# for idxImg in range(len(cam_26)):
#     img26 = cam_26[idxImg][1]
#     targetT = cam_26[idxImg][0]

#     while ps_24[idx24][0] < targetT: idx24 += 1
#     while ps_26[idx26][0] < targetT: idx26 += 1
#     while camInfo_26[idxInfo][0] < targetT: idxInfo += 1

#     mocapOffset = 0
#     if idx24 + mocapOffset == len(ps_24): break
#     if idx26 + mocapOffset == len(ps_26): break

#     ps24 = ps_24[idx24+mocapOffset][1]
#     ps26 = ps_26[idx26+mocapOffset][1]
#     camInfo26 = camInfo_26[idxInfo][1]
    
#     pos24 = ps24.position
#     ori24 = ps24.orientation
#     pos26 = ps26.position
#     ori26 = ps26.orientation

#     pt24_T_w = transformerROS.fromTranslationRotation((pos24.x, pos24.y, pos24.z), (ori24.x, ori24.y, ori24.z, ori24.w))
#     pt24_T_w = fixRotation(pt24_T_w)
#     pt26_T_w = transformerROS.fromTranslationRotation((pos26.x, pos26.y, pos26.z), (ori26.x, ori26.y, ori26.z, ori26.w))
#     pt26_T_w = fixRotation(pt26_T_w)

#     base24_T_w = np.matmul(pt24_T_w, baselink_T_trackedPt)
#     cam24_T_w = np.matmul(base24_T_w, colorCam_T_baselink)
#     base26_T_w = np.matmul(pt26_T_w, baselink_T_trackedPt)
#     cam26_T_w = np.matmul(base26_T_w, colorCam_T_baselink)

#     w_T_cam26 = inverse_transform(cam26_T_w)
#     cam24_T_cam26 = np.matmul(w_T_cam26, cam24_T_w)

#     # the Camera Pinhole model uses +x right, +y down, +z forward
#     pt_3d = (-cam24_T_cam26[1,3],-cam24_T_cam26[2,3],cam24_T_cam26[0,3])
#     camModel.fromCameraInfo(camInfo26)
#     pt_2d = camModel.project3dToPixel(pt_3d)
#     p2_round = (int(pt_2d[0]), int(pt_2d[1]))
    
#     car26_image = bridge.imgmsg_to_cv2(img26, "bgr8")
#     cv2.circle(car26_image, p2_round, 10, (0,255,0), 3)

#     combo = car26_image
#     if idxImg - 2 >= 0:
#         img26_1 = cam_26[idxImg-1][1]
#         img26_2 = cam_26[idxImg-2][1]
#         car26_image_1 = bridge.imgmsg_to_cv2(img26_1, "bgr8")
#         cv2.circle(car26_image_1, p2_round, 10, (0,255,0), 3)
#         car26_image_2 = bridge.imgmsg_to_cv2(img26_2, "bgr8")
#         cv2.circle(car26_image_2, p2_round, 10, (0,255,0), 3)
#         combo = cv2.hconcat([car26_image, car26_image_1])
#         combo = cv2.hconcat([combo, car26_image_2])

#     cv2.imwrite('combo_0_' + str(idxImg) + '.png', combo)

# idx24 = 0
# idx26 = 0
# idxInfo = 0
# for idxImg in range(len(cam_26)):
#     img26 = cam_26[idxImg][1]
#     targetT = cam_26[idxImg][0]

#     while ps_24[idx24][0] < targetT: idx24 += 1
#     while ps_26[idx26][0] < targetT: idx26 += 1
#     while camInfo_26[idxInfo][0] < targetT: idxInfo += 1

#     mocapOffset = -5
#     if idx24 + mocapOffset == len(ps_24): break
#     if idx26 + mocapOffset == len(ps_26): break

#     ps24 = ps_24[idx24+mocapOffset][1]
#     ps26 = ps_26[idx26+mocapOffset][1]
#     camInfo26 = camInfo_26[idxInfo][1]
    
#     pos24 = ps24.position
#     ori24 = ps24.orientation
#     pos26 = ps26.position
#     ori26 = ps26.orientation

#     pt24_T_w = transformerROS.fromTranslationRotation((pos24.x, pos24.y, pos24.z), (ori24.x, ori24.y, ori24.z, ori24.w))
#     pt24_T_w = fixRotation(pt24_T_w)
#     pt26_T_w = transformerROS.fromTranslationRotation((pos26.x, pos26.y, pos26.z), (ori26.x, ori26.y, ori26.z, ori26.w))
#     pt26_T_w = fixRotation(pt26_T_w)

#     base24_T_w = np.matmul(pt24_T_w, baselink_T_trackedPt)
#     cam24_T_w = np.matmul(base24_T_w, colorCam_T_baselink)
#     base26_T_w = np.matmul(pt26_T_w, baselink_T_trackedPt)
#     cam26_T_w = np.matmul(base26_T_w, colorCam_T_baselink)

#     w_T_cam26 = inverse_transform(cam26_T_w)
#     cam24_T_cam26 = np.matmul(w_T_cam26, cam24_T_w)

#     # the Camera Pinhole model uses +x right, +y down, +z forward
#     pt_3d = (-cam24_T_cam26[1,3],-cam24_T_cam26[2,3],cam24_T_cam26[0,3])
#     camModel.fromCameraInfo(camInfo26)
#     pt_2d = camModel.project3dToPixel(pt_3d)
#     p2_round = (int(pt_2d[0]), int(pt_2d[1]))
    
#     car26_image = bridge.imgmsg_to_cv2(img26, "bgr8")
#     cv2.circle(car26_image, p2_round, 10, (0,255,0), 3)

#     combo = car26_image
#     if idxImg - 2 >= 0:
#         img26_1 = cam_26[idxImg-1][1]
#         img26_2 = cam_26[idxImg-2][1]
#         car26_image_1 = bridge.imgmsg_to_cv2(img26_1, "bgr8")
#         cv2.circle(car26_image_1, p2_round, 10, (0,255,0), 3)
#         car26_image_2 = bridge.imgmsg_to_cv2(img26_2, "bgr8")
#         cv2.circle(car26_image_2, p2_round, 10, (0,255,0), 3)
#         combo = cv2.hconcat([car26_image, car26_image_1])
#         combo = cv2.hconcat([combo, car26_image_2])

#     cv2.imwrite('combo_1_' + str(idxImg) + '.png', combo)

# idx24 = 0
# idx26 = 0
# idxInfo = 0
# for idxImg in range(len(cam_26)):
#     img26 = cam_26[idxImg][1]
#     targetT = cam_26[idxImg][0]

#     while ps_24[idx24][0] < targetT: idx24 += 1
#     while ps_26[idx26][0] < targetT: idx26 += 1
#     while camInfo_26[idxInfo][0] < targetT: idxInfo += 1

#     mocapOffset = 5
#     if idx24 + mocapOffset == len(ps_24): break
#     if idx26 + mocapOffset == len(ps_26): break

#     ps24 = ps_24[idx24+mocapOffset][1]
#     ps26 = ps_26[idx26+mocapOffset][1]
#     camInfo26 = camInfo_26[idxInfo][1]
    
#     pos24 = ps24.position
#     ori24 = ps24.orientation
#     pos26 = ps26.position
#     ori26 = ps26.orientation

#     pt24_T_w = transformerROS.fromTranslationRotation((pos24.x, pos24.y, pos24.z), (ori24.x, ori24.y, ori24.z, ori24.w))
#     pt24_T_w = fixRotation(pt24_T_w)
#     pt26_T_w = transformerROS.fromTranslationRotation((pos26.x, pos26.y, pos26.z), (ori26.x, ori26.y, ori26.z, ori26.w))
#     pt26_T_w = fixRotation(pt26_T_w)

#     base24_T_w = np.matmul(pt24_T_w, baselink_T_trackedPt)
#     cam24_T_w = np.matmul(base24_T_w, colorCam_T_baselink)
#     base26_T_w = np.matmul(pt26_T_w, baselink_T_trackedPt)
#     cam26_T_w = np.matmul(base26_T_w, colorCam_T_baselink)

#     w_T_cam26 = inverse_transform(cam26_T_w)
#     cam24_T_cam26 = np.matmul(w_T_cam26, cam24_T_w)

#     # the Camera Pinhole model uses +x right, +y down, +z forward
#     pt_3d = (-cam24_T_cam26[1,3],-cam24_T_cam26[2,3],cam24_T_cam26[0,3])
#     camModel.fromCameraInfo(camInfo26)
#     pt_2d = camModel.project3dToPixel(pt_3d)
#     p2_round = (int(pt_2d[0]), int(pt_2d[1]))
    
#     car26_image = bridge.imgmsg_to_cv2(img26, "bgr8")
#     cv2.circle(car26_image, p2_round, 10, (0,255,0), 3)

#     combo = car26_image
#     if idxImg - 2 >= 0:
#         img26_1 = cam_26[idxImg-1][1]
#         img26_2 = cam_26[idxImg-2][1]
#         car26_image_1 = bridge.imgmsg_to_cv2(img26_1, "bgr8")
#         cv2.circle(car26_image_1, p2_round, 10, (0,255,0), 3)
#         car26_image_2 = bridge.imgmsg_to_cv2(img26_2, "bgr8")
#         cv2.circle(car26_image_2, p2_round, 10, (0,255,0), 3)
#         combo = cv2.hconcat([car26_image, car26_image_1])
#         combo = cv2.hconcat([combo, car26_image_2])

#     cv2.imwrite('combo_2_' + str(idxImg) + '.png', combo)

# # img align
# for i in range(2, len(cam_26)):
#     img1 = cv2.imread('/home/tudorf/mushr/combo_1_' + str(i) + '.png')
#     img2 = cv2.imread('/home/tudorf/mushr/combo_0_' + str(i) + '.png')
#     img3 = cv2.imread('/home/tudorf/mushr/combo_2_' + str(i) + '.png')
#     combo = cv2.vconcat([img1, img2])
#     combo = cv2.vconcat([combo, img3])
#     cv2.imwrite('combo' + str(i) + '.png', combo)