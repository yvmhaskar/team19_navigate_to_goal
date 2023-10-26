# Joseph Sommer and Yash Mhaskar
# Description: Subscribes to /odom data and the getObjectRange data and determines a heading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import time

# etc
import numpy as np
import math

#Global Variables
global state
state = 1

class Goal_Pub_Sub(Node):
	def __init__(self):
		super().__init__('goal_pub_sub')
		# State (for the update_Odometry code)
		self.Init = True
		self.Init_pos = Point()
		self.Init_pos.x = 0.0
		self.Init_pos.y = 0.0
		self.Init_ang = 0.0
		self.globalPos = Point()

		self.odom_sub = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			1)
		self.odom_sub  # prevent unused variable warning
		self.heading = self.create_publisher(Float32MultiArray,'heading',10)


	def odom_callback(self, data):
		self.update_Odometry(data)

	def update_Odometry(self,Odom):
		position = Odom.pose.pose.position
		global state
		#Orientation uses the quaternion aprametrization.
		#To get the angular position along the z-axis, the following equation is required.
		q = Odom.pose.pose.orientation
		orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

		if self.Init:
			#The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
			self.Init = False
			self.Init_ang = orientation
			self.globalAng = self.Init_ang
			Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
			self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
			self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
			self.Init_pos.z = position.z
		Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

		#We subtract the initial values
		self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
		self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
		self.globalAng = orientation - self.Init_ang
	
		self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))

		xodom = self.globalPos.x
		yodom = self.globalPos.y
		curPos = [xodom, yodom]
		theta_odom = self.globalAng
		lin_err_range = 0.05
		self.get_logger().info('State{}'.format(state))
	
		if state==1: # State 1: (0,0) to (1.5, 0)
			goal = [1.5, 0]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			self.get_logger().info('Lin Error{}'.format(e_l))

			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.heading.publish(msg)
			elif e_l < lin_err_range:
				state = 2
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.heading.publish(msg)
				time.sleep(10)
		elif state == 2: # State 2: (1.5, 0) to (2.25, 0.7)
			goal = [2.25, 0.7]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			self.get_logger().info('Lin Error{}'.format(e_l))

			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.heading.publish(msg)
			elif e_l < lin_err_range:
				state = 3
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.heading.publish(msg)
				time.sleep(10)
		elif state == 3: # State 3: (2.25, 0.7) to (1.5, 1.4)
			goal = [1.5, 1.4]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			self.get_logger().info('Lin Error{}'.format(e_l))

			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.heading.publish(msg)
			elif e_l < lin_err_range:
				state = 4
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.heading.publish(msg)
				time.sleep(10)
		elif state == 4: # State 4: (1.5, 1.4) to (0, 1.4)
			goal = [0, 1.4]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			self.get_logger().info('Lin Error{}'.format(e_l))

			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.heading.publish(msg)
			elif e_l < lin_err_range:
				state = 4
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.heading.publish(msg)
				time.sleep(10)
	
def main(args=None):
	rclpy.init(args=args)
	goal_pub_sub = Goal_Pub_Sub()
	rclpy.spin(goal_pub_sub)
	goal_pub_sub.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
	main()