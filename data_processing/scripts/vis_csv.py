"""
To visualize the push down (start time) of the cars run this script.
"""

import matplotlib.pyplot as plt
import csv
import pandas as pd

car24 = []
car25 = []
car26 = []
timestamp =[]

# Change the csv file name to whatever your file is
df = pd.read_csv('Take2_36min_2019-08-29 01.04.22 PM.csv', skiprows=6, low_memory=False)
for index, row in df.iterrows():
	if (row[1]>2000):
		timestamp.append(row[1])
		car24.append(row[7])
		car25.append(row[14])
		car26.append(row[21])
		
		#if row[1] > 50:
		#	break

plt.plot(timestamp, car24, label='car 24', color='b')
plt.plot(timestamp, car25, label='car 25', color='g')
plt.plot(timestamp, car26, label='car 26', color='r')
plt.xlabel('timestamp[s]')
plt.ylabel('car z axis[m]')
plt.title('visualizing push down')
plt.legend()
plt.show()

