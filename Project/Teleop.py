#! /usr/bin/env python3
import rclpy
import math
import numpy as np
import keyboard
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class Turtlebot3Teleop(Node):
    def __init__(self):
        self.enabled = True # continue teleop
        self.linearVelControl = 0.0 # linear control
        self.angularVelControl = 0.0 # angular control
        self.linearVelDesired = 0.0 # desired linear velocity
        self.angularVelDesired = 0.0 # desired angular velocity
        self.maxLinearVel = 2.25 # max linear control
        self.maxAngularVel = 2.25 # max angular control
        self.maxMotorControl = 2.25 # max motor control by hardware
        self.minMotorControl = -2.25 # min motor control by hardware
        self.rampTime = 0.25 # acceleration time in seconds
        self.sampleTime = 0.05 # time step between calcs

        self.timer = self.create_timer(self.sampleTime, self.run)


    def calculateControls(self):
        timeStep = (self.get_clock().now() - self.prev_time).nanoseconds / 1e-9
        linearAccel = self.maxLinearVel / self.rampTime
        angularAccel = self.maxLinearVel / self.rampTime

        # compute the next linear velocity under constant accel
        if self.linearVelControl - self.linearVelDesired != 0:
            self.linearVelControl = self.linearVelControl + linearAccel * timeStep * self.linearVelDesired / abs(self.linearVelDesired)

        # compute the next angular velocity under constant accel
        if self.angularVelControl - self.angularVelDesired != 0:
            self.angularVelControl = self.angularVelControl + angularAccel * timeStep * self.angularVelDesired / abs(self.angularVelDesired)

        # cap the control at the desired velocity
        if abs(self.linearVelControl) > abs(self.linearVelDesired):
            self.linearVelControl = self.linearVelDesired

        if abs(self.angularVelControl) > abs(self.angularVelDesired):
            self.angularVelControl = self.angularVelDesired

        # cap the control at the max motor velocity
        if self.linearVelControl > self.maxMotorControl:
            self.linearVelControl = self.maxMotorControl

        elif self.linearVelControl < self.minMotorControl:
            self.linearVelControl = self.minMotorControl

        if self.linearVelControl > self.maxMotorControl:
            self.linearVelControl = self.maxMotorControl
            
        elif self.linearVelControl < self.minMotorControl:
            self.linearVelControl = self.minMotorControl
    def recInput(self):
        # determine desired forwared velocity
        if keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
            self.linearVelDesired = self.maxLinearVel
        elif keyboard.is_pressed('s') and not keyboard.is_pressed('w'):
            self.linearVelDesired = -self.maxLinearVel
        else:
            self.linearVelDesired = 0

        # determine desired angular velocity
        if keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
            self.angularVelDesired = self.maxAngularVel
        elif keyboard.is_pressed('d') and not keyboard.is_pressed('a'):
            self.angularVelDesired = -self.maxAngularVel
        else:
            self.angularVelDesired = 0

        # shutdown cleanly
        if keyboard.is_pressed('esc'):
            print("Initiating Shutdown!")
            self.linearVelControl = 0
            self.angularVelControl = 0
            self.enabled = False

    def run(self):
        self.recInput() # determine desired control state
        if self.enabled:
            self.calculateControls() # calculate command to achieve state

        twist = Twist() # declare control interface
        twist.linear.x = self.linearVelControl # set linear velocity
        twist.angular.z = self.angularVelControl # set angular velocity
        self.cmd_vel_pub.publish(twist) # publish data to robot node



def main(args=None):
    rclpy.init(args=args)
    controller = Turtlebot3Teleop()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()