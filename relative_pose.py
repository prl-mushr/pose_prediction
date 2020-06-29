import rosbag
import numpy as np

import tf
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

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

transformerROS = tf.TransformerROS()

bag = rosbag.Bag('output.bag')
topics = ['/car24/PoseStamped','/car26/PoseStamped']
car24PoseStamps = []
car26PoseStamps = []
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == '/car24/PoseStamped':
        car24PoseStamps.append((msg, t))
    elif topic == '/car26/PoseStamped':
        car26PoseStamps.append((msg, t))


idx26 = 0
for idx24, (msg, t) in enumerate(car24PoseStamps):
    # bring car26 data into sync
    while idx26 < len(car26PoseStamps) - 1 and t > car26PoseStamps[idx26][1]:
        idx26 += 1

    car24PoseStamp = msg
    car26PoseStamp = car26PoseStamps[idx26][0]

    car24Pose = car24PoseStamp.pose
    car24_T_w = transformerROS.fromTranslationRotation((car24Pose.position.x, car24Pose.position.y, car24Pose.position.z),
        (car24Pose.orientation.x, car24Pose.orientation.y, car24Pose.orientation.z, car24Pose.orientation.w))
    
    car26Pose = car26PoseStamp.pose
    car26_T_w = transformerROS.fromTranslationRotation((car26Pose.position.x, car26Pose.position.y, car26Pose.position.z),
        (car26Pose.orientation.x, car26Pose.orientation.y, car26Pose.orientation.z, car26Pose.orientation.w))

    w_T_car26 = inverse_transform(car26_T_w)
    car26_T_car24 = np.matmul(w_T_car26,car24_T_w)

    print('point_w.r.t._reference')
    print('car24_T_w', car24_T_w)
    print('car26_T_w', car26_T_w)
    print('w_T_car26', w_T_car26)
    print('car26_T_car24', car26_T_car24)
    
    break  # exit after first loop