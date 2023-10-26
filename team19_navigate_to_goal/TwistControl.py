# Joseph Sommer and Yash Mhaskar

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy
from simple_pid import PID

class TwistSubPub(Node):
	def __init__(self):
		super().__init__('twist_sub_pub')
		self.subscription = self.create_subscription(Float32MultiArray, 'heading', self.mover_callback, 5)
		self.subscription
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)

	def mover_callback(self, msg):
		mover_data = msg.data
		e_l = float(mover_data[0])
		e_a = float(mover_data[1])
		e_a = e_a*3.14159/180
		e_a = numpy.arctan2(numpy.sin(e_a), numpy.cos(e_a))

		setlin = 0.0
		setang = 0.0
		#bufferlin = 0.1
		#bufferang = 0.2		

		pid_l = PID(-1,0.000,0.0,setpoint=setlin)
		v_l = pid_l(e_l)
		pid_a = PID(-0.6,0.0,0.0,setpoint=setang)
		v_a = pid_a(e_a)
		
		if v_l > 0.22:
			v_l = 0.2
		elif v_l < -0.22:
			v_l = -0.2
		if v_a > 1.42:
			v_a = 1.42
		elif v_a < -1.42:
			v_a = -1.42
		
		twist = Twist()
		if abs(e_a) > 0.1:
			self.get_logger().info('Adjusting Angular twist')
			#self.get_logger().info('e_l: "%s"'% e_l)
			#self.get_logger().info('v_l: "%s"'% v_l)
			twist.angular.z = v_a
			twist.linear.x = 0.0
			
		elif abs(e_a) < 0.11:
			self.get_logger().info('Adjusting Linear twist')
			#self.get_logger().info('e_l: "%s"'% e_l)
			#self.get_logger().info('v_l: "%s"'% v_l)
			twist.angular.z = 0.0
			twist.linear.x = v_l

		if abs(e_l)<0.05:
			self.get_logger().info('Reached point')
			#self.get_logger().info('e_a: "%s"'% e_a)
			#self.get_logger().info('e_l: "%s"'% e_l)
			twist.angular.z = 0.0
			twist.linear.x = 0.0
			
		
		self.cmd_vel.publish(twist)

def main(args=None):

	rclpy.init(args=args)
	twist_sub_pub = TwistSubPub()
	rclpy.spin(twist_sub_pub)
	twist_sub_pub.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
