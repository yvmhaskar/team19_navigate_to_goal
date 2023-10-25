# brings in odom global coordinate
import time
Global state = 1; 

class MovementSubPub(Node):
	def __init__(self):
		super().__init__('movement_sub_pub')
		self.subscription = self.create_subscription(Float32MultiArray, 'movement_coord', self.mover_callback, 5)
		self.subscription
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)

	def mover_callback(self, msg):		
		xodom = 0.5
		yodom = 0
		curPos = [xodom, yodom]
		theta_odom = 2.6
		twist = Twist()
		lin_err_range = 0.05
	
		if state==1: # State 1: (0,0) to (1.5, 0)
			goal = [1.5, 0]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.movement_publisher.publish(msg)
			elif e_l < lin_err_range:
				state = 2
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.movement_publisher.publish(msg)
				time.sleep(10)
		elif state == 2: # State 2: (1.5, 0) to (2.25, 0.7)
			goal = [2.25, 0.7]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.movement_publisher.publish(msg)
			elif e_l < lin_err_range:
				state = 3
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.movement_publisher.publish(msg)
				time.sleep(10)
		elif state == 3: # State 3: (2.25, 0.7) to (1.5, 1.4)
			goal = [1.5, 1.4]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.movement_publisher.publish(msg)
			elif e_l < lin_err_range:
				state = 4
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.movement_publisher.publish(msg)
				time.sleep(10)
		elif state == 4: # State 4: (1.5, 1.4) to (0, 1.4)
			goal = [0, 1.4]
			e_l = distance_to_goal = math.sqrt((goal[0] - curPos[0]) ** 2 + (goal[1] - curPos[1]) ** 2)
			e_a = math.atan2(goal[1] - curPos[1], goal[0] - curPos[0]) - theta_odom
			if e_l > lin_err_range:
				msg = Float32MultiArray()
				msg.data = [e_l,e_a]
				# Publish the x-axis position
				self.movement_publisher.publish(msg)
			elif e_l < lin_err_range:
				state = 4
				e_l = 0
				e_a = 0
				# publish e_l and e_a self.cmd_vel.publish(twist)
				msg = Float32MultiArray()
				msg.data = [e_l, e_a]
				self.movement_publisher.publish(msg)
				time.sleep(10)
