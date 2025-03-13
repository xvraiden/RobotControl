# sudo apt -get install ros-foxy-tf-transformations
# sudo pip3 install transformations3d


#! /usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import sys

class Turtlebot3PIDController(Node):

	def __init__(self):
		super().__init__('turtlebot_subscriber')
		print("Turtlebot Subscriber Node Initialized")

		#Robot initial States
		self.x = 0
		self.y = 0
		self.theta = 0
		self.linear_vel = .3
		self.angular_vel = 0
		self.radius = 5.0
		self.counter = 0
		self.running = 1.0


		#PID gains
		self.Kp = 4 #Reaction to current error
		self.Ki = 2E-20 #SS correction, too high means past is too important
		self.Kd = 100 #Damping on the error (also velocity)

		#PID initial parameters
		self.error = 0
		self.derivative = 0.0
		self.integral = 0.0
		self.prev_err = 0.0
		self.int_err = 1.0
		self.prevTime = self.get_clock().now()
		self.prev_time = self.get_clock().now()

		self.distThreshold = 0.1 #distance threshold for waypoint navigation
		self.maxVelError = 0.1
		self.maxMotorVel = 2.34
		self.minMotorVel = -2.34
		self.motorVel = 0

		self.angVelTopic = "/leftMotorVel"
		self.angVelTopicType = "Float32"

		self.rate = 10 #10hz
		self.odomData = None #empty variable

		self.odomTopic = '/odom'

		#Data publishing and subscription
		self.create_subscription(Odometry, '/odom', self.odom_callback, 10)# subscribe to the odometry data and when messages are received, self.odom_callback is invoked # NEW
		self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)# publishing velocity data # NEW
		self.err = Float32()

		#coordinate waypoints
		#example

		#########WAYPOINT SETTINGS
		self.waypoints = np.array([[1,1], [5,5], [9,1], [3,-5]])
		#########

		self.waypoint_size = self.waypoints.shape[0]

		#sinusoidal waypoint
		self.amplitude = 1
		self.frequency = 2/9 #cycles/xunit
		self.maxT = 9
		self.numSinSteps = 100
		self.sinSteps = np.empty((self.numSinSteps,2))

		for i in range(self.numSinSteps):
			newStep = np.array([(i+1)*(self.maxT)*(1/self.numSinSteps), self.amplitude*np.sin(2 * np.pi * self.frequency * (i+1)*(self.maxT)*(1/self.numSinSteps))])
			self.sinSteps[i] = [newStep[1], newStep[0]]

		self.waypoint_counter = 0
		self.maxMotorVel = 2.84
		self.minMotorVel = -2.84

		self.timer = self.create_timer(0.1, self.run)

	def odom_callback(self, msg): # Initiate odometry data
		self.odomData = msg

	def getOrientation(self): #get the yaw angle by converting rotations to euler angles
		orientation_q = self.odomData.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

		self.theta = yaw
		return self.theta

	def saturate(self, input, delta): #function to saturate the output from the PID controller inside an acceptable range
		if(input > self.maxMotorVel):
			control = self.maxMotorVel
			self.int_err = self.int_err - (self.err * delta)

		elif(input < self.minMotorVel):
			control = self.minMotorVel
			self.int_err = self.int_err - (self.err * delta)
		else:
			control = input
		return control

	def calculate_error(self):
		yaw = self.getOrientation()

		desiredAngle = np.arctan2(self.goal[1] - self.y, self.goal[0] - self.x)

		path1 = desiredAngle - yaw
		path2 = desiredAngle - (np.pi - yaw)

		if (abs(path1) > abs(path2)):
			self.err = path2
		else:
			self.err = path1

		#self.err = np.pi / 2 - yaw #tuning code

		print(f"Im at {self.x, self.y}\n")
		print(f"Im heading to {self.goal[0], self.goal[1]}\n")
		print(f"My error is {self.err}\n")
		return self.err

	def pid_controller(self):
		current_error = self.calculate_error() # call the output from the calculate_error() function to use it in the controller
		dt = (self.get_clock().now() - self.prev_time).nanoseconds / 1e-9
		# if dt == 0:
		# 	return 0
		delta = 0

		# take the current area undre the area curve and add the new error area
		self.error = self.error + (current_error * dt)

		# take the derivative of the rate of change of error
		self.derivative = (current_error - self.prev_err) / dt

		# define easier
		self.integral = self.error
	
		# construct PID
		control = (self.Kp * current_error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

		# ensure we dont overload the motors
		control = self.saturate(control, delta)

		# keep track of last error
		self.prev_err = current_error

		return control

	def run(self):
		self.rate = self.create_rate(10)
		self.prev_time=self.get_clock().now()
		if self.odomData is not None:
			#goal - waypoints
			if (self.waypoint_counter < self.waypoints.shape[0]):
				# example-coordinate waypoints
				print("Box")
				wp_x = self.waypoints[self.waypoint_counter][0]
				wp_y = self.waypoints[self.waypoint_counter][1]
				self.goal = np.array([wp_x, wp_y])
			else:
				print("Sine")
				#sinusoidal
				wp_x = self.sinSteps[self.waypoint_counter - self.waypoints.shape[0]][0]
				wp_y = self.sinSteps[self.waypoint_counter - self.waypoints.shape[0]][1]
				self.goal = np.array([wp_x, wp_y])


			self.x = self.odomData.pose.pose.position.x
			self.y = self.odomData.pose.pose.position.y

			current_state = np.array([self.x, self.y])
			self.calculate_error()

			self.distError = np.linalg.norm(self.goal - current_state) #norm of the difference between two vectors

			if(self.distError > self.distThreshold): #the controller runs for a particular waypoint until it reaches a certain threshold
				dummy = 0
			else: #once the threshold is reached, we go to the next waypoint
				self.waypoint_counter +=1
				if(self.waypoint_counter == self.waypoints.shape[0] + self.sinSteps.shape[0]): #if it is the last waypoint, we go to the first waypoint from there
					self.linear_vel = 0.0
					self.running = 0.0
					print("Completed")

			twist = Twist()
			twist.linear.x = self.linear_vel
			#twist.linear.y = 0.3
			# print(control)
			twist.angular.z = self.pid_controller() * self.running

			self.cmd_vel_pub.publish(twist)
			#self.rate.sleep()

			if (self.running == 0.0):
				exit()


def main(args=None):
    rclpy.init(args=args)
    controller = Turtlebot3PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
