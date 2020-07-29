import rosbag
import numpy as np

import tf
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

import rospy

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
center_T_baselink = translate_transform([-0.015, 0.0, 0.003])

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

idx24 = 0
idx26 = 0
idxInfo = 0
images = []
img_size = (0, 0)
delay = 0.1
for idxImg in range(len(cam_26)):
    if idxImg == 60: break
    targetT = cam_26[idxImg][0]
    while idx26< len(ps_26)-1   and ps_26[idx26][0]     < targetT + rospy.Duration.from_sec(delay): idx26 += 1
    while idx24< len(ps_24)-1   and ps_24[idx24][0]     < targetT + rospy.Duration.from_sec(delay): idx24 += 1
    while idxImg< len(cam_26)-1 and cam_26[idxImg][0]   < targetT:                                  idxImg += 1

    # extract mocap pose data, cameraInfo, and images
    ps24 = ps_24[idx24][1]
    ps26 = ps_26[idx26][1]
    camInfo26 = camInfo_26[idxInfo][1]
    img26 = cam_26[idxImg][1]
    
    # unwrap mocap pose into position, orientation
    pos24 = ps24.position
    ori24 = ps24.orientation
    pos26 = ps26.position
    ori26 = ps26.orientation

    # create Transforms for TrackedPts w.r.t. World
    pt24_T_w = transformerROS.fromTranslationRotation((pos24.x, pos24.y, pos24.z), (ori24.x, ori24.y, ori24.z, ori24.w))
    pt24_T_w = fixRotation(pt24_T_w)
    pt26_T_w = transformerROS.fromTranslationRotation((pos26.x, pos26.y, pos26.z), (ori26.x, ori26.y, ori26.z, ori26.w))
    pt26_T_w = fixRotation(pt26_T_w)

    # create Transforms for points of interest w.r.t. World
    base24_T_w = np.matmul(pt24_T_w, baselink_T_trackedPt)
    center24_T_w = np.matmul(base24_T_w, center_T_baselink)
    base26_T_w = np.matmul(pt26_T_w, baselink_T_trackedPt)
    cam26_T_w = np.matmul(base26_T_w, colorCam_T_baselink)

    # create Transforms for points of interest w.r.t. Camera
    w_T_cam26 = inverse_transform(cam26_T_w)
    center24_T_cam26 = np.matmul(w_T_cam26, center24_T_w)

    # create Transforms for bounding box corners
    corners = [
        np.matmul(center24_T_cam26, translate_transform([ 0.22,  0.134,  0.079])),
        np.matmul(center24_T_cam26, translate_transform([ 0.22, -0.134,  0.079])),
        np.matmul(center24_T_cam26, translate_transform([ 0.22,  0.134, -0.079])),
        np.matmul(center24_T_cam26, translate_transform([ 0.22, -0.134, -0.079])),
        np.matmul(center24_T_cam26, translate_transform([-0.22,  0.134,  0.079])),
        np.matmul(center24_T_cam26, translate_transform([-0.22, -0.134,  0.079])),
        np.matmul(center24_T_cam26, translate_transform([-0.22,  0.134, -0.079])),
        np.matmul(center24_T_cam26, translate_transform([-0.22, -0.134, -0.079]))
    ]

    # find pixel coordinates of centroid
    # the Camera Pinhole model uses +x right, +y down, +z forward
    center_3d = (-center24_T_cam26[1,3], -center24_T_cam26[2,3], center24_T_cam26[0,3])
    camModel.fromCameraInfo(camInfo26)
    center_2d = camModel.project3dToPixel(center_3d)
    center_round = (int(center_2d[0]), int(center_2d[1]))
    # reject all future steps (drawing) on frame if the observed robot is behind the camera
    if center_3d[2] < 0: continue
    # convert to OpenCV image
    car26_image = bridge.imgmsg_to_cv2(img26, "bgr8")
    # draw centroid
    cv2.circle(car26_image, center_round, 5, (0,255,0), 3)

    # initialize rectangle bounds to image size
    # cv's image.shape is formatted as (height, width, length) a.k.a. (y, x, z)
    xmin = car26_image.shape[1]
    xmax = 0
    ymin = car26_image.shape[0]
    ymax = 0

    for corneridx, corner in enumerate(corners):
        # find pixel coordinates of corners
        corner_3d = (-corner[1,3], -corner[2,3], corner[0,3])
        corner_2d = camModel.project3dToPixel(corner_3d)
        corner_round = (int(corner_2d[0]), int(corner_2d[1]))
        # draw the front (first 4) corners in different colors
        color = (255,255,0)
        if corneridx > 3:
            color = (0,255,255)
        cv2.circle(car26_image, corner_round, 3, color, 2)

        xmin = min(xmin, corner_round[0])
        xmax = max(xmax, corner_round[0])
        ymin = min(ymin, corner_round[1])
        ymax = max(ymax, corner_round[1])

    # draw rectangle
    cv2.rectangle(car26_image, (xmin, ymin), (xmax, ymax), (0,0,255), 1)

    # Add to processed frames list
    images.append(car26_image)
    h, w, l = car26_image.shape
    img_size = (h, w)

# create video from processed frames
vid_out = cv2.VideoWriter('centroid_' + str(delay) + '.avi', cv2.VideoWriter_fourcc(*'MJPG'), 1, (640,480))
for i in range(len(images)):
    vid_out.write(images[i].astype('uint8'))
vid_out.release()
