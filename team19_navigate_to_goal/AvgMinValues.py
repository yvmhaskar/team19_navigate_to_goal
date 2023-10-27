# Find minimum 5 values from lidar data range

import numpy as np

def _lidar_callback(self,msg):
	global lidar_data
	global got_lidar
	lidar_data = msg.ranges
	default_range = 5
	for i in range(len(lidar_data)):
		if math.isnan(lidar_data[i]) or (lidar_data[i]==0):
			lidar_data[i] = default_range
	got_lidar = 1
	self.pub_coord()
	
def pub_coord(self):
	global x1
	global r
	global got_dir
	global got_lidar
	global counter
	
	global ang_err_old	
    # this part will access lidar range data for specific angles
	angular_resolution = 1.62 # degrees
	theta2 = 20*3.1415/180 # right edge
	theta1 = 0
	theta3 = 340*3.14159/180 # left edge
	theta4 = 2*3.1415 # center
	angle_index1 = int(theta1 / angular_resolution) # index that refers to angle of center of object
	angle_index2 = int(theta2 / angular_resolution) # index that refers to angle of edge of object
	angle_index3 = int(theta3 / angular_resolution) # index that refers to angle of edge of object
	angle_index4 = int(theta4 / angular_resolution) # index that refers to angle of edge of object

	# grab values in front between 340 - 360 and 0 - 20 and combine them
    length1 = angle_index4 - angle_index3
    frontLeft = [None] * length1
	for i in range(0, length, 1):
		frontLeft[i] = lidar_data[angle_index3+i]
		
    length2 = angle_index2 - angle_index1
    frontRight = [None] * length2
	for i in range(0, length, 1):
		frontRight[i] = lidar_data[angle_index1+i]
	
    frontData = np.concatenate((frontLeft,frontRight))
    k = 5 # number of vals taken in
    idx = np.argpartition(frontData, k)
    minFrontData = frontData[idx[:k]]
    minDist = np.average(minFrontData)
	

A = np.array([1, 7, 9, 2, 0.1, 17, 17, 1.5])
k = 3

idx = np.argpartition(A, k)
print(idx)
# [4 0 7 3 1 2 6 5]