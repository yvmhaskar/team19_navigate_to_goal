# Joseph Sommer and Yash Mhaskar
# Joseph Sommer and Yash Mhaskar

from __future__ import print_function
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math

global x1
global r
r = 0
global lidar_data
global got_lidar
got_lidar = 0
global counter
counter = 0
global ang_err_old
ang_err_old=0

class ObjectRangePubsub(Node):
	def __init__(self):
		super().__init__('object_range_pub_sub')

		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

		#Declare get_lidar node is subcribing to the /scan topic.
		self.get_lidar = self.create_subscription(
			LaserScan,
			'scan',
			self._lidar_callback,
			image_qos_profile)
		self.get_lidar # Prevents unused variable warning.

		#Declare dir_publisher
		self.obj_dist = self.create_publisher(Float32MultiArray, 'obj_dist', 10)
		#self.timer = self.create_timer(0.5, self.publish_command)


	

	def _lidar_callback(self,msg):
		lidar_data = msg.ranges
		lidar_length = len(lidar_data)
		if len(lidar_data) > 200:
			default_range = 20
			for i in range(len(lidar_data)):
				if math.isnan(lidar_data[i]) or (lidar_data[i]==0):
					lidar_data[i] = default_range
			self.get_logger().info('Lidar_length{}'.format(len(lidar_data)))
			#self.get_logger().info('Lidar_length{}'.format(lidar_data))
			##########
			angular_resolution = 1.62 # degrees
			theta1 = 0.0
			theta2 = 90.0 # right edge
			theta3 = 270.0 # left edge
			theta4 = 360.0 # center
			angle_index1 = int(theta1 / angular_resolution) # index that refers to angle of center of object
			angle_index2 = int(theta2 / angular_resolution) # index that refers to angle of edge of object
			angle_index3 = int(theta3 / angular_resolution) # index that refers to angle of edge of object
			angle_index4 = int(theta4 / angular_resolution) # index that refers to angle of edge of object
			
			angle_index4 = len(lidar_data)
			# grab values in front between 340 - 360 and 0 - 20 and combine them
			length1 = angle_index4 - angle_index3-1
			frontLeft = [None] * length1
			for i in range(0, length1, 1):
				frontLeft[i] = lidar_data[angle_index3+i]
			
			length2 = angle_index2 - angle_index1
			frontRight = [None] * length2
			for i in range(0, length2, 1):
				frontRight[i] = lidar_data[angle_index1+i]
			
			frontData = np.concatenate((frontLeft,frontRight))
			k = 5 # number of vals taken in
			idx = np.argpartition(frontData, k)
			
			minFrontData = frontData[idx[:k]]
			self.get_logger().info('idx: {}'.format(idx[:k]))

			if np.average(idx[:k])>=length1:
				turn_dir = 1.0
			else:
				turn_dir = -1.0
			closest_dist = np.average(minFrontData)
			msg = Float32MultiArray()
			msg.data = [closest_dist,turn_dir]
			self.obj_dist.publish(msg)
			self.get_logger().info('Closest_dist{}'.format(closest_dist))
			self.get_logger().info('Length1: {}'.format(length2))
			self.get_logger().info('turn: {}'.format(turn_dir))
		else:
			self.get_logger().info('Lidar_length{}'.format(len(lidar_data)))

def main(args=None):
	# Setting up publisher values
	rclpy.init(args=args)
	object_range_pub_sub=ObjectRangePubsub()
	rclpy.spin(object_range_pub_sub)

	rclpy.shutdown()