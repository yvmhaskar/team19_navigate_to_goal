# Joseph Sommer and Yash Mhaskar
# Description: Subscribes to /odom data and the getObjectRange data and determines a heading

# Import required Libraries
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from timeit import default_timer as timer
import numpy as np
import math

#Global Variables
global state
state = 1
global waiter
waiter = 0
global cur_time
cur_time = 0
global wait_time
wait_time = 0
global angReached
angReached = 0
global closest_obj
closest_obj = 0
global Obj_det_x
Obj_det_x = 1.0
global turn_dir
turn_dir = 1
global turning_dir


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

		# Subscribe to odometry data
		self.odom_sub = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			1)
		self.odom_sub  # prevent unused variable warning

		#Subscribe to getObjectRange data
		self.obj_dist_sub = self.create_subscription(
			Float32MultiArray,
			'/obj_dist',
			self.obj_dist_callback,
			1)
		self.heading = self.create_publisher(Float32MultiArray,'heading',10)


	def odom_callback(self, data):
		self.goToGoal(data)

	def obj_dist_callback(self, msg):
		global closest_obj
		global turn_dir
		msg_data = msg.data
		closest_obj = float(msg_data[0])
		turn_dir = float(msg_data[1])


	def rotate_direction(self, goal, curPos, theta_odom, theta_goal):			
		global angReached
		ang_err_range = 0.1
		a_offset = 0.1
		# ignore e_l to ensure only rotate first
		e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom + a_offset
#		e_a =  theta_goal- theta_odom
		e_a = np.arctan2(np.sin(e_a), np.cos(e_a))

		
		self.get_logger().info('Ang Error{}'.format(e_a))
		if abs(e_a) > ang_err_range:
			self.get_logger().info('Rotating')
		elif abs(e_a) <= ang_err_range:
			angReached = 1 # means true
			self.get_logger().info('Done Rotating')
			e_a = 0.0
		return e_a
		# publish e_l and e_a
		#msg = Float32MultiArray()
		#msg.data = [e_l, e_a]
		#self.heading.publish(msg)

	def goToGoal(self,Odom):
		position = Odom.pose.pose.position
		global waiter
		global state
		global cur_time
		global wait_time
		global angReached
		global Obj_det_x
		global closest_obj
		global turn_dir
		global turning_dir

		if waiter == 1:
			now_time = timer()
			if now_time - cur_time > wait_time:
				waiter = 0
				wait_time = 0
				cur_time= now_time
			self.get_logger().info('Sleeping')
		elif waiter == 0:
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
			err_offset = 1.8/1.5

			if state==1: # State 1: (0,0) to (1.5, 0)
				goal = [1.8, 0.0]
				theta_odom = theta_odom
				e_l = goal[0] - curPos[0] # Only X displacement
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 1')
				elif e_l <= lin_err_range:
					state = 2
					e_l = 0.0
					e_a = 0.0
					cur_time = timer()
					waiter = 1
					wait_time = 10

			elif state==2: # State 2: (1.5,0) to (1.7, 0)
				goal = [2.3, 0.0]
				e_l = goal[0] - curPos[0] # Only X displacement
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 2')
				elif e_l <= lin_err_range:
					state = 3
					e_l = 0.0
					e_a = 0.0

			elif state==3: # State 3: rotate 90 degrees CCW
				goal = [2.3, 1.7]
				theta_goal = 3.14/2.0 # 90 degrees
				theta_odom = theta_odom
				e_l = 0.0
				e_a = 0.0
				if angReached==0:
					self.get_logger().info('STATE 3')
					e_a=self.rotate_direction(goal, curPos, theta_odom, theta_goal)
				elif angReached==1:
					state=4
					angReached=0
			
			elif state==4: # State 4: (1.7,0) to (1.7, 1.4)
				goal = [2.2, 1.7]
				e_l = goal[1] - curPos[1] # Only Y displacement
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 4')
				elif e_l <= lin_err_range:
					state = 5
					e_l = 0.0
					e_a = 0.0
			
			elif state==5: # State 5: rotate 90 degrees CCW
				goal = [1.7, 1.7]
				theta_goal = 3.14159265 # 90 degrees
				theta_odom = theta_odom
				e_l = 0.0
				e_a = 0.0
				if angReached==0:
					self.get_logger().info('STATE 5')
					e_a=self.rotate_direction(goal, curPos, theta_odom, theta_goal)
				elif angReached==1:
					state=6
					angReached=0
			
			elif state==6: # State 6: (1.7,1.4) to (1.5, 1.4)
				goal = [1.7, 1.7]
				e_l = curPos[0] - goal[0]
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 6')
				elif e_l <= lin_err_range:
					state = 7
					e_l = 0.0
					e_a = 0.0
					cur_time = timer()
					waiter = 1
					wait_time = 10
			
			elif state==7: # State 7: (1.5,1.4) to (1.3, 1.4)
				goal = [0.0, 1.8]
				e_l = curPos[0] - goal[0]
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if closest_obj<=0.3:
					state=8
					e_l = 0.0
					e_a = 0.0
					Obj_det_x = curPos[0]
					turning_dir = turn_dir
					self.get_logger().info('Turning_dir: {}'.format(turning_dir))
				elif e_l > lin_err_range:
					self.get_logger().info('STATE 7')
				elif e_l <= lin_err_range:
					state = 14
					e_l = 0.0
					e_a = 0.0
			
			elif state==8: # State 8: rotate 90 degrees CW

				goal = [Obj_det_x-0.1, 1.8+turning_dir*0.5]
				theta_goal = 3.18/2 # 90 degrees
				theta_odom = theta_odom
				e_l = 0.0
				e_a = 0.0
				if angReached==0:
					self.get_logger().info('STATE 8')
					e_a=self.rotate_direction(goal, curPos, theta_odom, theta_goal)
				elif angReached==1:
					state=9
					angReached=0
			
			elif state==9: # State 9: (1.3,1.4) to (1.3, 1.8)
				goal = [Obj_det_x, 1.8+turning_dir*0.5]
				e_l = abs(goal[1] - curPos[1])
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 9')
				elif e_l <= lin_err_range:
					state = 10
					e_l = 0.0
					e_a = 0.0

			elif state==10: # State 10: rotate 90 degrees CCW
				goal = [0.0, 1.8+turning_dir*0.5]
				theta_goal = 3.14159265 # 180 degrees
				theta_odom = theta_odom
				e_l = 0.0
				e_a = 0.0
				if angReached==0:
					self.get_logger().info('STATE 10')
					e_a=self.rotate_direction(goal, curPos, theta_odom, theta_goal)
				elif angReached==1:
					state=11
					angReached=0
			
			elif state==11: # State 11: (1.3,1.8) to (0, 1.8)
				goal = [0.0, 1.8+turning_dir*0.5]
				e_l = curPos[0] - goal[0]
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 11')
				elif e_l <= lin_err_range:
					state = 12
					e_l = 0.0
					e_a = 0.0
			
			elif state==12: # State 12: rotate 90 degrees CCW
				goal = [0.0, 1.7]
				theta_goal = -3.14159265/2 # 180 degrees
				theta_odom = theta_odom
				e_l = 0.0
				e_a = 0.0
				if angReached==0:
					self.get_logger().info('STATE 12')
					e_a=self.rotate_direction(goal, curPos, theta_odom, theta_goal)
				elif angReached==1:
					state=13
					angReached=0

			elif state==13: # State 13: (0.0, 1.8) to (0, 1.4)
				goal = [0.0, 1.7]
				e_l = abs(curPos[1] - goal[1])
				e_a = 0.0 #math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
				if e_l > lin_err_range:
					self.get_logger().info('STATE 13')
				elif e_l <= lin_err_range:
					state = 14
					e_l = 0.0
					e_a = 0.0
					cur_time = timer()
					waiter = 1
					wait_time = 10

			elif state==14: # State 14: Reached point
				goal = [0.0, 1.8]
				e_l = 0.0
				e_a = 0.0
				self.get_logger().info('STATE 14')

			self.get_logger().info('x_goal{}'.format(goal[0]))
			self.get_logger().info('y_goal{}'.format(goal[1]))
			self.get_logger().info('Lin Errornow{}'.format(e_l))
			self.get_logger().info('Ang Errornow{}'.format(e_a))
			# publish e_l and e_a
			msg = Float32MultiArray()
			msg.data = [e_l, e_a]
			self.heading.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	goal_pub_sub = Goal_Pub_Sub()
	rclpy.spin(goal_pub_sub)
	goal_pub_sub.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
	main()
