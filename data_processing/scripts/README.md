# Steps to create time-synced bag file of three cars 

1. Use vis_csv.py to help figure our the pushdown(start times) of each car and get that timestamp is at least ms precision. 
2. Using these timestamps and csv2bag.py, create three bag files(each containing information from 1 car) from the csv file. (This script also fixed the coordinates). Look at comments in the script for detail on how to produce a bag for each car.
	
	python csv2bag.py 

3. Run topic_rename.py for each car bag file to rename their topics to avoid collision when we merge the 3 bags later on.
	
	python topic_rename.py <inbag> <outbag> <car#>

3. Look at the three start time stamps for the three cars and pick the car that start the latest. This is going to be the start time for when we merge all three bags files for each car.
4. Now for the 2 cars that have earlier start times, run this command to filter the bag by timestamp to get it to the same time stamp as the latest timestamp car. 
for this example timestamp = 1564521265.01000000
	
	rosbag filter <inbag> <outbag> "t.secs >= 1564521265.010000000"

5. Now the bag files should have the same start time stamps.
6. Run merge_bag.py to merge the three bag files
	
	python topic_rename.py <inbag> <outbag> <car#>

7. Now you should have a time-synced bag file 
8. Check by playing the bag and comparing images to visualized poses. 
