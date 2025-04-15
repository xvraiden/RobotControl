#! /usr/bin/env python3
import rclpy
import math
import numpy as np
from pynput import keyboard
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class Turtlebot3Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot_subscriber')
        print("Turtlebot Subscriber Node Initialized")

        self.enabled = True # continue teleop

        self.linearVelControl = 0.0 # linear control
        self.angularVelControl = 0.0 # angular control
        self.linearVelDesired = 0.0 # desired linear velocity
        self.angularVelDesired = 0.0 # desired angular velocity
        self.linearAccel = 0.0 # current linear accel
        self.angularAccel = 0.0 # current angular accel
        self.linearCoeffs = np.zeros(shape=(4, 1)) # coefficients for cubic
        self.angularCoeffs = np.zeros(shape=(4, 1)) # coefficients for cubic
        self.linearTime = 0.0 # time since linear trajectory calc
        self.angularTime = 0.0  # time since angular trajectory calc

        self.maxLinearVel = 1.125 # max linear control
        self.maxAngularVel = 1.125 # max angular control

        self.maxMotorControl = 2.25 # max motor control by hardware
        self.minMotorControl = -2.25 # min motor control by hardware

        self.rampTime = 0.125 # acceleration time in seconds
        self.sampleTime = 0.01 # time step between calcs
        self.prev_time = self.get_clock().now()

        self.forwardPress = False # forward keyboard command active
        self.backwardPress = False # backward keyboard command active
        self.leftPress = False # left keyboard command active
        self.rightPress = False # right keyboard command active
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release).start()


        self.rate = 1 / self.sampleTime  # 10hz

        # Data publishing and subscription
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', int(self.rate))  # publishing velocity data # NEW
        self.err = Float32()

        self.timer = self.create_timer(self.sampleTime, self.run)

    def on_press(self, key):
        if key.char == 'w' and not self.forwardPress:
            self.linearVelDesired = self.linearVelDesired + self.maxLinearVel
            self.computeTrajectory("linear")
            self.forwardPress = True
        elif key.char == 's' and not self.backwardPress:
            self.linearVelDesired = self.linearVelDesired - self.maxLinearVel
            self.computeTrajectory("linear")
            self.backwardPress = True
        elif key.char == 'a' and not self.leftPress:
            self.angularVelDesired = self.angularVelDesired + self.maxAngularVel
            self.computeTrajectory("angular")
            self.leftPress = True
        elif key.char == 'd' and not self.rightPress:
            self.angularVelDesired = self.angularVelDesired - self.maxAngularVel
            self.computeTrajectory("angular")
            self.rightPress = True

    def on_release(self, key):
        if key.char == 'w' and self.forwardPress:
            self.linearVelDesired = self.linearVelDesired - self.maxLinearVel
            self.computeTrajectory("linear")
            self.forwardPress = False
        elif key.char == 's' and self.backwardPress:
            self.linearVelDesired = self.linearVelDesired + self.maxLinearVel
            self.computeTrajectory("linear")
            self.backwardPress = False
        elif key.char == 'a' and self.leftPress:
            self.angularVelDesired = self.angularVelDesired - self.maxAngularVel
            self.computeTrajectory("angular")
            self.leftPress = False
        elif key.char == 'd' and self.rightPress:
            self.angularVelDesired = self.angularVelDesired + self.maxAngularVel
            self.computeTrajectory("angular")
            self.rightPress = False

    def computeTrajectory(self, axis):
        # compute trajectory equation
        A1 = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [1, self.rampTime, self.rampTime ** 2, self.rampTime ** 3],
             [0, 1, 2 * self.rampTime, 3 * self.rampTime ** 2]])

        if axis == "linear":
            b1 = np.array([[self.linearVelControl], [self.linearAccel], [self.linearVelDesired], [0]])
            self.linearCoeffs = np.matmul(np.linalg.inv(A1), b1)
            self.linearTime = 0.0
        else:
            b1 = np.array([[self.angularVelControl], [self.angularAccel], [self.angularVelDesired], [0]])
            self.angularCoeffs = np.matmul(np.linalg.inv(A1), b1)
            self.angularTime = 0.0

    def calculateControls(self):
        # get time difference between runs
        currentTime = self.get_clock().now()
        timeStep = (currentTime - self.prev_time).nanoseconds / 1e9
        self.prev_time = currentTime

        # compute velocity and acceleration at current time
        if self.linearTime <= self.rampTime:
            self.linearVelControl = float(self.linearCoeffs[3] * self.linearTime ** 3 + self.linearCoeffs[2] * self.linearTime ** 2 + self.linearCoeffs[1] * self.linearTime + self.linearCoeffs[0])
            self.linearAccel = float(3 * self.linearCoeffs[3] * self.linearTime ** 2 + 2 * self.linearCoeffs[2] * self.linearTime + self.linearCoeffs[1])
            self.linearTime = self.linearTime + timeStep
        else:
            self.linearVelControl = self.linearVelDesired
            self.linearAccel = 0

        if self.angularTime <= self.rampTime:
            self.angularVelControl = float(self.angularCoeffs[3] * self.angularTime ** 3 + self.angularCoeffs[2] * self.angularTime ** 2 + self.angularCoeffs[1] * self.angularTime + self.angularCoeffs[0])
            self.angularAccel = float(3 * self.angularCoeffs[3] * self.angularTime ** 2 + 2 * self.angularCoeffs[2] * self.angularTime +self.angularCoeffs[1])
            self.angularTime = self.angularTime + timeStep
        else:
            self.angularVelControl = self.angularVelDesired
            self.angularAccel = 0

    def obstacleOverride(self):
        print("obstacle override")

    def limitControls(self):
        # cap the control at the max motor velocity
        if self.linearVelControl > self.maxMotorControl:
            self.linearVelControl = self.maxMotorControl

        elif self.linearVelControl < self.minMotorControl:
            self.linearVelControl = self.minMotorControl

        if self.angularVelControl > self.maxMotorControl:
            self.angularVelControl = self.maxMotorControl
            
        elif self.angularVelControl < self.minMotorControl:
            self.angularVelControl = self.minMotorControl

    def run(self):
        if self.enabled:
            self.obstacleOverride() # override user input to avoid hitting stuff
            self.calculateControls() # calculate command to achieve state
            self.limitControls() # ensure command states dont exceed motor capabilities

        print(self.linearVelControl)
        print(self.angularVelControl)

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
