#! /usr/bin/env python3
import rclpy
import numpy as np
import math
from pynput import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class Turtlebot3Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot_subscriber')
        print("Turtlebot Subscriber Node Initialized")

        # enable control
        self.enabled = True # continue teleop

        # control variables
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

        # control limits
        self.maxLinearVel = 0.5 # max linear control
        self.maxAngularVel = 1.125 # max angular control

        # hardware limits
        self.maxMotorControl = 2.25 # max motor control by hardware
        self.minMotorControl = -2.25 # min motor control by hardware

        # time systems
        self.rampTime = 0.125 # acceleration time in seconds
        self.prev_time = self.get_clock().now()

        # keyboard states
        self.forwardPress = False # forward keyboard command active
        self.backwardPress = False # backward keyboard command active
        self.leftPress = False # left keyboard command active
        self.rightPress = False # right keyboard command active
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release).start()

        # obstacle detection params
        self.cautionRadius = 0.5 # slow down when obstacle is closer than this [m]
        self.cautionSpeed = 0.25 # percentage of full speed when close to obstacle
        self.stopRadius = 0.35 # initiate safe stop when obstacle is closer than this[m]
        self.emergencyRadius = 0.2 # initiate emergency stop when obstacle is closer than this [m]
        self.angularInterest = 15 # angle to include data off the front and rear [deg]
        self.frontState = "none" # state of front obstacle detection
        self.rearState = "none"  # state of rear obstacle detection

        # Data publishing and subscription
        self.rate = 10 # 10hz
        self.scanData = None  # empty variable
        self.scanTopic = '/scan'
        self.create_subscription(LaserScan, self.scanTopic, self.scan_callback, 5)  # subscribe to the odometry data and when messages are received, self.odom_callback is invoked # NEW
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.rate)  # publishing velocity data # NEW
        self.err = Float32()

        self.timer = self.create_timer(1 / self.rate, self.run)

    def scan_callback(self, msg):  # Initiate scan data
        print("Getting data")
        self.scanData = msg

    def on_press(self, key):
        try:
            if key.char == 'w' and not self.forwardPress:
                self.linearVelDesired = self.linearVelDesired + self.maxLinearVel
                self.forwardPress = True
                self.augmentCommand("linear")
            elif key.char == 's' and not self.backwardPress:
                self.linearVelDesired = self.linearVelDesired - self.maxLinearVel
                self.backwardPress = True
                self.augmentCommand("linear")
            elif key.char == 'a' and not self.leftPress:
                self.angularVelDesired = self.angularVelDesired + self.maxAngularVel
                self.leftPress = True
                self.augmentCommand("angular")
            elif key.char == 'd' and not self.rightPress:
                self.angularVelDesired = self.angularVelDesired - self.maxAngularVel
                self.rightPress = True
                self.augmentCommand("angular")
        except:
            dummy = 0

    def on_release(self, key):
        try:
            if key.char == 'w' and self.forwardPress:
                self.linearVelDesired = self.linearVelDesired - self.maxLinearVel
                self.forwardPress = False
                self.augmentCommand("linear")
            elif key.char == 's' and self.backwardPress:
                self.linearVelDesired = self.linearVelDesired + self.maxLinearVel
                self.backwardPress = False
                self.augmentCommand("linear")
            elif key.char == 'a' and self.leftPress:
                self.angularVelDesired = self.angularVelDesired - self.maxAngularVel
                self.leftPress = False
                self.augmentCommand("angular")
            elif key.char == 'd' and self.rightPress:
                self.angularVelDesired = self.angularVelDesired + self.maxAngularVel
                self.rightPress = False
                self.augmentCommand("angular")
        except:
            dummy = 0

    def augmentCommand(self, axis):
        if axis == "linear":
            if self.linearVelDesired > 0:
                if self.frontState == "caution":
                    augmentedVel = self.linearVelDesired * self.cautionSpeed
                elif self.frontState == "stop":
                    augmentedVel = 0.0
                elif self.frontState == "emergency":
                    augmentedVel = 0.0
                else:
                    augmentedVel = self.linearVelDesired

            else:
                if self.rearState == "caution":
                    augmentedVel = self.linearVelDesired * self.cautionSpeed
                elif self.rearState == "stop":
                    augmentedVel = 0.0
                elif self.rearState == "emergency":
                    augmentedVel = 0.0
                else:
                    augmentedVel = self.linearVelDesired

            self.computeTrajectory("linear", augmentedVel)
        else:
            self.computeTrajectory("angular", self.angularVelDesired)

    def computeTrajectory(self, axis, vel):
        # compute trajectory equation
        A1 = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [1, self.rampTime, self.rampTime ** 2, self.rampTime ** 3],
             [0, 1, 2 * self.rampTime, 3 * self.rampTime ** 2]])

        if axis == "linear":
            b1 = np.array([[self.linearVelControl], [self.linearAccel], [vel], [0]])
            self.linearCoeffs = np.matmul(np.linalg.inv(A1), b1)
            self.linearTime = 0.0
        else:
            b1 = np.array([[self.angularVelControl], [self.angularAccel], [vel], [0]])
            self.angularCoeffs = np.matmul(np.linalg.inv(A1), b1)
            self.angularTime = 0.0

    def obstacleOverride(self):
        if self.scanData is not None:
            # determine indices of interest
            indexFrontLeftStart = 0
            indexFrontLeftEnd = int((self.angularInterest * math.pi / 180) / self.scanData.angle_increment)
            indexFrontRightStart = int((2 * math.pi - self.angularInterest * math.pi / 180) / self.scanData.angle_increment)
            indexFrontRightEnd = len(self.scanData.ranges)

            indexRearStart = int((math.pi - self.angularInterest * math.pi / 180) / self.scanData.angle_increment)
            indexRearEnd = int((math.pi + self.angularInterest * math.pi / 180) / self.scanData.angle_increment)

            # get data of interest
            frontDistances = self.scanData.ranges[indexFrontLeftStart:indexFrontLeftEnd] + self.scanData.ranges[indexFrontRightStart:indexFrontRightEnd]
            rearDistances = self.scanData.ranges[indexRearStart:indexRearEnd]

            if self.linearVelControl >= 0:
                if min(frontDistances) < self.emergencyRadius:
                    if self.frontState != "emergency":
                        self.frontState = "emergency"
                        print("front emergency stop")
                elif min(frontDistances) < self.stopRadius:
                    if self.frontState != "stop":
                        self.frontState = "stop"
                        self.augmentCommand("linear")
                        print("front stop")
                elif min(frontDistances) < self.cautionRadius:
                    if self.frontState != "caution":
                        self.frontState = "caution"
                        self.augmentCommand("linear")
                        print("front caution")
                else:
                    if self.frontState != "none":
                        self.frontState = "none"
                        self.augmentCommand("linear")

            if self.linearVelControl <= 0:
                if min(rearDistances) < self.emergencyRadius:
                    if self.rearState != "emergency":
                        self.rearState = "emergency"
                        print("rear emergency stop")
                elif min(rearDistances) < self.stopRadius:
                    if self.rearState != "stop":
                        self.rearState = "stop"
                        self.augmentCommand("linear")
                        print("rear stop")
                elif min(rearDistances) < self.cautionRadius:
                    if self.rearState != "caution":
                        self.rearState = "caution"
                        self.augmentCommand("linear")
                        print("rear caution")
                else:
                    if self.rearState != "none":
                        self.rearState = "none"
                        self.augmentCommand("linear")

    def calculateControls(self):
        # get time difference between runs
        currentTime = self.get_clock().now()
        timeStep = (currentTime - self.prev_time).nanoseconds / 1e9
        self.prev_time = currentTime

        # compute velocity and acceleration at current time
        if self.linearTime <= self.rampTime:
            self.linearVelControl = round(float(self.linearCoeffs[3] * self.linearTime ** 3 + self.linearCoeffs[2] * self.linearTime ** 2 + self.linearCoeffs[1] * self.linearTime + self.linearCoeffs[0]), 2)
            self.linearAccel = round(float(3 * self.linearCoeffs[3] * self.linearTime ** 2 + 2 * self.linearCoeffs[2] * self.linearTime + self.linearCoeffs[1]), 2)
            self.linearTime = self.linearTime + timeStep
        else:
            self.linearVelControl = round(float(self.linearCoeffs[3] * self.rampTime ** 3 + self.linearCoeffs[2] * self.rampTime ** 2 + self.linearCoeffs[1] * self.rampTime + self.linearCoeffs[0]), 2)
            self.linearAccel = 0

        if self.angularTime <= self.rampTime:
            self.angularVelControl = round(float(self.angularCoeffs[3] * self.angularTime ** 3 + self.angularCoeffs[2] * self.angularTime ** 2 + self.angularCoeffs[1] * self.angularTime + self.angularCoeffs[0]), 2)
            self.angularAccel = round(float(3 * self.angularCoeffs[3] * self.angularTime ** 2 + 2 * self.angularCoeffs[2] * self.angularTime +self.angularCoeffs[1]), 2)
            self.angularTime = self.angularTime + timeStep
        else:
            self.angularVelControl = round(float(self.angularCoeffs[3] * self.rampTime ** 3 + self.angularCoeffs[2] * self.rampTime ** 2 + self.angularCoeffs[1] * self.rampTime + self.angularCoeffs[0]), 2)
            self.angularAccel = 0

        self.computeEmergency() # determine if we are decelerating in emergency stop

    def computeEmergency(self):
        if self.linearVelControl > 0 and self.frontState == "emergency":
            self.linearVelControl = 0.0
            self.linearAccel = 0.0

        elif self.linearVelControl < 0 and self.rearState == "emergency":
            self.linearVelControl = 0.0
            self.linearAccel = 0.0

        self.limitControls()  # ensure command states dont exceed motor capabilities

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
        print(self.scanData)
        if self.enabled and self.scanData is not None:
            self.obstacleOverride() # override user input to avoid hitting stuff
            self.calculateControls() # calculate input to drive system

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
