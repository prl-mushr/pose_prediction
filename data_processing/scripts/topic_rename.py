#!/usr/bin/env python
"""
Renames list of topics in bag
	
python topic_rename.py <inbag> <outbag> <car#>

List of Topics
 /camera/accel/imu_info                               1 msg     : realsense2_camera/IMUInfo            
             /camera/accel/sample                            568417 msgs    : sensor_msgs/Imu                      
             /camera/color/camera_info                        72347 msgs    : sensor_msgs/CameraInfo               
             /camera/color/image_throttled                    26288 msgs    : sensor_msgs/Image                    
             /camera/depth/camera_info                        67373 msgs    : sensor_msgs/CameraInfo               
             /camera/depth/image_rect_throttled               25590 msgs    : sensor_msgs/Image                    
             /camera/extrinsics/depth_to_color                    1 msg     : realsense2_camera/Extrinsics         
             /camera/gyro/imu_info                                1 msg     : realsense2_camera/IMUInfo            
             /camera/gyro/sample                             899064 msgs    : sensor_msgs/Imu                      
             /camera/motion_module/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription
             /camera/motion_module/parameter_updates              1 msg     : dynamic_reconfigure/Config           
             /camera/realsense2_camera_manager/bond            4494 msgs    : bond/Status                           (2 connections)
             /camera/rgb_camera/parameter_descriptions            1 msg     : dynamic_reconfigure/ConfigDescription
             /camera/rgb_camera/parameter_updates                 1 msg     : dynamic_reconfigure/Config           
             /camera/stereo_module/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription
             /camera/stereo_module/parameter_updates              1 msg     : dynamic_reconfigure/Config           
             /car_pose                                        44943 msgs    : geometry_msgs/PoseStamped            
             /dev/null                                            2 msgs    : std_msgs/Int8                        
             /diagnostics                                      6637 msgs    : diagnostic_msgs/DiagnosticArray       (2 connections)
             /joint_states                                    44937 msgs    : sensor_msgs/JointState               
             /mux/ackermann_cmd_mux/active                        1 msg     : std_msgs/String                      
             /mux/ackermann_cmd_mux/input/default             13482 msgs    : ackermann_msgs/AckermannDriveStamped 
             /mux/ackermann_cmd_mux/input/teleop              74194 msgs    : ackermann_msgs/AckermannDriveStamped 
             /mux/ackermann_cmd_mux/output                    74198 msgs    : ackermann_msgs/AckermannDriveStamped 
             /mux/ackermann_cmd_mux/parameter_descriptions        1 msg     : dynamic_reconfigure/ConfigDescription
             /mux/ackermann_cmd_mux/parameter_updates             1 msg     : dynamic_reconfigure/Config           
             /mux/nodelet_manager/bond                         4493 msgs    : bond/Status                           (2 connections)
             /push_button_state                              224411 msgs    : std_msgs/Bool                        
             /rosout                                             25 msgs    : rosgraph_msgs/Log                     (8 connections)
             /rosout_agg                                         22 msgs    : rosgraph_msgs/Log                    
             /scan                                            19574 msgs    : sensor_msgs/LaserScan                
             /teleop/joy                                      74219 msgs    : sensor_msgs/Joy                      
             /tf                                             134811 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           2 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /vesc/commands/motor/speed                      272086 msgs    : std_msgs/Float64                     
             /vesc/commands/motor/unsmoothed_speed            74200 msgs    : std_msgs/Float64                     
             /vesc/commands/servo/position                   168410 msgs    : std_msgs/Float64                     
             /vesc/commands/servo/unsmoothed_position         74206 msgs    : std_msgs/Float64                     
             /vesc/odom                                      112335 msgs    : nav_msgs/Odometry                    
             /vesc/sensors/core                              112340 msgs    : vesc_msgs/VescStateStamped           
             /vesc/sensors/servo_position_command            168450 msgs    : std_msgs/Float64


"""
from rosbag import Bag
import rospy
import sys


inbag = sys.argv[1]
outbag = sys.argv[2]
whichCar = str(sys.argv[3])

topic_names = ['/camera/accel/sample', '/camera/color/camera_info', '/camera/color/image_throttled', '/camera/depth/camera_info',
		'/camera/depth/image_rect_throttled', '/camera/extrinsics/depth_to_color', '/camera/gyro/imu_info', '/camera/gyro/sample',
		'/camera/motion_module/parameter_descriptions', '/camera/motion_module/parameter_updates', '/camera/realsense2_camera_manager/bond',
		'/camera/rgb_camera/parameter_descriptions' , '/camera/rgb_camera/parameter_updates', '/camera/stereo_module/parameter_descriptions',
		'/camera/stereo_module/parameter_updates', '/car_pose', '/dev/null','/diagnostics', '/joint_states', '/mux/ackermann_cmd_mux/active',
		'/mux/ackermann_cmd_mux/input/default', '/mux/ackermann_cmd_mux/input/teleop', '/mux/ackermann_cmd_mux/output',
		'/mux/ackermann_cmd_mux/parameter_descriptions', '/mux/ackermann_cmd_mux/parameter_updates','/mux/nodelet_manager/bond' ,
		'/push_button_state', '/rosout', '/rosout_agg', '/scan', '/teleop/joy', '/tf', '/tf_static', '/vesc/commands/motor/speed',
		'/vesc/commands/motor/unsmoothed_speed', '/vesc/commands/servo/position', '/vesc/commands/servo/unsmoothed_position', 
		'/vesc/odom', '/vesc/sensors/core', '/vesc/sensors/servo_position_command']

print(len(topic_names))

with Bag(outbag, 'w') as namedBag:
    for topic, msg, t in Bag(inbag):
	for name in topic_names: 
	    if (topic == name):
	        namedBag.write('/'+whichCar+name, msg, t)
			 


