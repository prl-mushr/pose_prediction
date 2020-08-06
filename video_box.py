import rosbag
import numpy as np
import tf
import rospy
import cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

PATH_TO_INPUT_BAG = '/home/tudorf/mushr/catkin_ws/src/learning-image-geometry/car37_longtrim.bag'
OUTPUT_VIDEO_NAME = 'test'
OUTPUT_VIDEO_FPS = 9

transformerROS = tf.TransformerROS()
bridge = CvBridge()
camModel = PinholeCameraModel()
vid_out = cv2.VideoWriter(OUTPUT_VIDEO_NAME + '.avi', cv2.VideoWriter_fourcc(*'MJPG'), OUTPUT_VIDEO_FPS, (640,480))

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

trackedPt_T_baselink = translate_transform([-0.058325, 0, 0.08125])
colorCam_T_baselink = translate_transform([0.02, 0.033, 0.068])
center_T_baselink = translate_transform([-0.015, 0.0, 0.003])
baselink_T_trackedPt = inverse_transform(trackedPt_T_baselink)

bag = rosbag.Bag(PATH_TO_INPUT_BAG)
topics = ['/vrpn_client_node/car35/pose', '/vrpn_client_node/car37/pose','/vrpn_client_node/car38/pose','/car37/camera/color/camera_info','/car37/camera/color/image_throttled']

ps_35 = []
ps_37 = []
ps_38 = []
camInfo = None
cam_37 = []

for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/vrpn_client_node/car35/pose':
        ps_35.append((t, msg.pose))
    elif topic == '/vrpn_client_node/car37/pose':
        ps_37.append((t, msg.pose))
    elif topic == '/vrpn_client_node/car38/pose':
        ps_38.append((t, msg.pose))
    elif topic == '/car37/camera/color/camera_info':
        camInfo = msg
    elif topic == '/car37/camera/color/image_throttled':
        cam_37.append((t, msg))

idx35 = 0
idx37 = 0
idx38 = 0

print('mocap35', len(ps_35))
print('mocap37', len(ps_37))
print('mocap38', len(ps_38))
print('image37', len(cam_37))

for idxImg in range(len(cam_37)):
    targetT = cam_37[idxImg][0]
    while idx35< len(ps_35)-1   and ps_35[idx35][0]   < targetT: idx35 += 1
    while idx37< len(ps_37)-1   and ps_37[idx37][0]   < targetT: idx37 += 1
    while idx38< len(ps_38)-1   and ps_38[idx38][0]   < targetT: idx38 += 1

    # pick closest mocap data, not next
    if idx35 > 0 and targetT - ps_35[idx35-1][0] < ps_35[idx35][0] - targetT:
        idx35 -= 1
    if idx37 > 0 and targetT - ps_37[idx37-1][0] < ps_37[idx37][0] - targetT:
        idx37 -= 1
    if idx38 > 0 and targetT - ps_38[idx38-1][0] < ps_38[idx38][0] - targetT:
        idx38 -= 1
    
    # unwrap mocap pose into position, orientation
    pos35 = ps_35[idx35][1].position
    ori35 = ps_35[idx35][1].orientation
    pos37 = ps_37[idx37][1].position
    ori37 = ps_37[idx37][1].orientation
    pos38 = ps_38[idx38][1].position
    ori38 = ps_38[idx38][1].orientation

    # create Transforms for TrackedPts w.r.t. World
    pt35_T_w = transformerROS.fromTranslationRotation((pos35.x, pos35.y, pos35.z), (ori35.x, ori35.y, ori35.z, ori35.w))
    pt37_T_w = transformerROS.fromTranslationRotation((pos37.x, pos37.y, pos37.z), (ori37.x, ori37.y, ori37.z, ori37.w))
    pt38_T_w = transformerROS.fromTranslationRotation((pos38.x, pos38.y, pos38.z), (ori38.x, ori38.y, ori38.z, ori38.w))

    # create Transforms for points of interest w.r.t. World
    base37_T_w = np.matmul(pt37_T_w, baselink_T_trackedPt)
    cam37_T_w = np.matmul(base37_T_w, colorCam_T_baselink)

    base35_T_w = np.matmul(pt35_T_w, baselink_T_trackedPt)
    center35_T_w = np.matmul(base35_T_w, center_T_baselink)
    base38_T_w = np.matmul(pt38_T_w, baselink_T_trackedPt)
    center38_T_w = np.matmul(base38_T_w, center_T_baselink)

    # create Transforms for points of interest w.r.t. Camera
    w_T_cam37 = inverse_transform(cam37_T_w)
    center38_T_cam37 = np.matmul(w_T_cam37, center38_T_w)
    center35_T_cam37 = np.matmul(w_T_cam37, center35_T_w)

    # create Transforms for bounding box corners
    corners35 = [
        np.matmul(center35_T_cam37, translate_transform([ 0.22,  0.134,  0.079])),
        np.matmul(center35_T_cam37, translate_transform([ 0.22, -0.134,  0.079])),
        np.matmul(center35_T_cam37, translate_transform([ 0.22,  0.134, -0.079])),
        np.matmul(center35_T_cam37, translate_transform([ 0.22, -0.134, -0.079])),
        np.matmul(center35_T_cam37, translate_transform([-0.22,  0.134,  0.079])),
        np.matmul(center35_T_cam37, translate_transform([-0.22, -0.134,  0.079])),
        np.matmul(center35_T_cam37, translate_transform([-0.22,  0.134, -0.079])),
        np.matmul(center35_T_cam37, translate_transform([-0.22, -0.134, -0.079]))
    ]
    corners38 = [
        np.matmul(center38_T_cam37, translate_transform([ 0.22,  0.134,  0.079])),
        np.matmul(center38_T_cam37, translate_transform([ 0.22, -0.134,  0.079])),
        np.matmul(center38_T_cam37, translate_transform([ 0.22,  0.134, -0.079])),
        np.matmul(center38_T_cam37, translate_transform([ 0.22, -0.134, -0.079])),
        np.matmul(center38_T_cam37, translate_transform([-0.22,  0.134,  0.079])),
        np.matmul(center38_T_cam37, translate_transform([-0.22, -0.134,  0.079])),
        np.matmul(center38_T_cam37, translate_transform([-0.22,  0.134, -0.079])),
        np.matmul(center38_T_cam37, translate_transform([-0.22, -0.134, -0.079]))
    ]

    # convert to OpenCV image
    car37_image = bridge.imgmsg_to_cv2(cam_37[idxImg][1], "bgr8")

    # find pixel coordinates of centroids
    # the Camera Pinhole model uses +x right, +y down, +z forward
    center35_3d = (-center35_T_cam37[1,3], -center35_T_cam37[2,3], center35_T_cam37[0,3])
    center38_3d = (-center38_T_cam37[1,3], -center38_T_cam37[2,3], center38_T_cam37[0,3])
    camModel.fromCameraInfo(camInfo)
    center35_2d = camModel.project3dToPixel(center35_3d)
    center38_2d = camModel.project3dToPixel(center38_3d)
    center35_round = (int(center35_2d[0]), int(center35_2d[1]))
    center38_round = (int(center38_2d[0]), int(center38_2d[1]))


    # only draw on frame if the observed robot is behind the camera
    car35_rect = None
    if center35_3d[2] > 0:
        # draw centroid
        cv2.circle(car37_image, center35_round, 3, (0,255,0), 2)

        # initialize rectangle bounds to image size
        # cv's image.shape is formatted as (height, width, length) a.k.a. (y, x, z)
        xmin = car37_image.shape[1] + 1
        xmax = -1
        ymin = car37_image.shape[0] + 1
        ymax = -1

        for corneridx, corner in enumerate(corners35):
            # find pixel coordinates of corners
            corner_3d = (-corner[1,3], -corner[2,3], corner[0,3])
            corner_2d = camModel.project3dToPixel(corner_3d)
            corner_round = (int(corner_2d[0]), int(corner_2d[1]))
            # draw the front (first 4) corners in different colors
            color = (255,255,0)
            cv2.circle(car37_image, corner_round, 2, color, 1)

            xmin = min(xmin, corner_round[0])
            xmax = max(xmax, corner_round[0])
            ymin = min(ymin, corner_round[1])
            ymax = max(ymax, corner_round[1])

        # save rectangle
        car35_rect = ((xmin, ymin), (xmax, ymax))
    car38_rect = None
    if center38_3d[2] > 0:
        # draw centroid
        cv2.circle(car37_image, center38_round, 3, (0,255,0), 2)

        # cv's image.shape is formatted as (height, width, length) a.k.a. (y, x, z)
        xmin = car37_image.shape[1] + 1
        xmax = -1
        ymin = car37_image.shape[0] + 1
        ymax = -1

        for corneridx, corner in enumerate(corners38):
            # find pixel coordinates of corners
            corner_3d = (-corner[1,3], -corner[2,3], corner[0,3])
            corner_2d = camModel.project3dToPixel(corner_3d)
            corner_round = (int(corner_2d[0]), int(corner_2d[1]))
            # draw the front (first 4) corners in different colors
            color = (0,255,255)
            cv2.circle(car37_image, corner_round, 2, color, 1)

            xmin = min(xmin, corner_round[0])
            xmax = max(xmax, corner_round[0])
            ymin = min(ymin, corner_round[1])
            ymax = max(ymax, corner_round[1])

        # save rectangle
        car38_rect = ((xmin, ymin), (xmax, ymax))

    # draw rectangles & check for overlap
    overlap_tolerance = 5
    if car35_rect is not None:
        if car38_rect is not None:
            if car35_rect[0][0] > car38_rect[0][0] - overlap_tolerance and car35_rect[0][1] > car38_rect[0][1] - overlap_tolerance and \
                car35_rect[1][0] < car38_rect[1][0] + overlap_tolerance and car35_rect[1][1] < car38_rect[1][1] + overlap_tolerance:
                    # reject draw
                    print('car38 overlaps car35')
            else:
                cv2.rectangle(car37_image, car35_rect[0], car35_rect[1], (0,0,255), 1)
        else:
            cv2.rectangle(car37_image, car35_rect[0], car35_rect[1], (0,0,255), 1)
    
    if car38_rect is not None:
        if car35_rect is not None:
            if car38_rect[0][0] > car35_rect[0][0] - overlap_tolerance and car38_rect[0][1] > car35_rect[0][1] - overlap_tolerance and \
                car38_rect[1][0] < car35_rect[1][0] + overlap_tolerance and car38_rect[1][1] < car35_rect[1][1] + overlap_tolerance:
                    # reject draw
                    print('car35 overlaps car38')
            else:
                cv2.rectangle(car37_image, car38_rect[0], car38_rect[1], (0,0,255), 1)
        else:
            cv2.rectangle(car37_image, car38_rect[0], car38_rect[1], (0,0,255), 1)
    

    # Add frame to video
    vid_out.write(car37_image.astype('uint8'))

# end video
vid_out.release()