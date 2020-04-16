	
"""
rotate position coord and quarternion coord in x-axis by 90 and Publish contents to a bagfile at each timestamp. 

For future use:
1. Get the timestamps for the start times of each car. 
2. Change the start_time and start timestamps as reminded in the comments. 
3. This script has to be ran once for each car. So 3 times for three car. 

"""

import rospy
from rosbag import Bag
import pandas as pd
import time
import std_msgs.msg
import sys
import numpy as np 
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped

# change the name of the csv file to whatever new csv file is
df = pd.read_csv('Take2_36min_2019-08-29 01.04.22 PM.csv', skiprows=6, low_memory=False)

start_time = 14.1522 # -REQUIRES CHANGE- duration(seconds) from the start of recording to pushdown(start of movement). Different for each car.
start_timestamp = 1564521138.363700000 # -REQUIRES CHANGE- timestamp of the push down. Different for each car.
time_incr = 0.008333 #in sec
target_bag = sys.argv[1]
numTopics = 21
rotation_matrix = np.array([[ 1, 0, 0], 
                 	 		[ 0, 0,-1], 
                  	 		[ 0, 1, 0]])

quat_rot_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
				 		   [0.0, 0.0,-1.0, 0.0],
				 		   [0.0, 1.0, 0.0, 0.0],
				 	       [0.0, 0.0, 0.0, 1.0]])


q8d = Quaternion(matrix = quat_rot_matrix)

goal = PoseStamped()

with Bag(target_bag, 'w') as outbag:
	for index, row in df.iterrows():
		# Only start parsing csv upon start time 
		if row[1] >= start_time:
			# Getting desired timestamp and converting it to time
			start_timestamp = start_timestamp + time_incr
			timestamp = rospy.Time(start_timestamp)
			
			## Position Coordinate rotation around x-axis 90 degrees 
			car24_pos_vector = np.array([row[6],row[7],row[8]]) 
			Rot_car24_pos_vector = rotation_matrix.dot(car24_pos_vector) 
			car25_pos_vector = np.array([row[13],row[14],row[15]])
			Rot_car25_pos_vector = rotation_matrix.dot(car25_pos_vector)
			car26_pos_vector = np.array([row[20],row[21],row[22]])
			Rot_car26_pos_vector = rotation_matrix.dot(car26_pos_vector)
			
			## Quaternion Coordinate rotation around x-axis 90 degrees 
			car24_quat_vector = np.array([row[5], row[2], row[3], row[4]])
			q6 = Quaternion(car24_quat_vector)
			rctv24 = q8d*q6
			car25_quat_vector = np.array([row[12], row[9], row[10], row[11]])
			q7 = Quaternion(car25_quat_vector)
			rctv25 = q8d*q7
			car26_quat_vector = np.array([row[19], row[16], row[17], row[18]])
			q8 = Quaternion(car26_quat_vector)
			rctv26 = q8d*q8
			
			# Converting fixed pose data to ROS topic 'posestamped'
			# change Rot_car25_pos_vector and rctv25[0] to corresponding car numbers
			goal.header.frame_id = '/world'
			goal.pose.position.x = Rot_car25_pos_vector[0]
			goal.pose.position.y = Rot_car25_pos_vector[1]
			goal.pose.position.z = Rot_car25_pos_vector[2]
			goal.pose.orientation.w = rctv25[0]
			goal.pose.orientation.x = rctv25[1]
			goal.pose.orientation.y = rctv25[2]
			goal.pose.orientation.z = rctv25[3]


			# writing to a bag file. Change name to corresponding car
			outbag.write("/car25/PoseStamped", goal, timestamp)
	
