# sudo apt -get install ros-foxy-tf-transformations
# sudo pip3 install transformations3d


# ! /usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt


class Turtlebot3PIDController(Node):

    def __init__(self):
        super().__init__('turtlebot_subscriber')
        print("Turtlebot Subscriber Node Initialized")

        # PID gains
        self.Kp = 2  # Reaction to current error
        self.Ki = 0.0  # SS correction, too high means past is too important
        self.Kd = 0.2 # Damping on the error (also velocity)

        # Waypoint params
        self.waypointParcerSize = 0.15 # distance between each waypoint
        self.distThreshold = 0.05  # distance threshold for waypoint navigation
        self.obstacleThreshold = 70 # cutoff for what is and isnt an obstacle #65 was historical

        # Force tuning coeff
        self.attractiveForceCoeff = 1  # attractive force scalar
        self.repulsiveForceCoeff = 1  # repulsive force scalar
        self.stepSizeCoeff = 0.01  # step size per iteration

        #Static TF map-> odom
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)

        # Distance thresholds
        self.attractiveSwitch = 0.1  # distance where attractive force becomes linear
        self.repulsiveSwitch = 0.3  # distance at which repulsive is negligible
        self.acceptableTargetError = self.distThreshold  # distance at which the robot is close enough to target
        self.notMovingDist = 0.75 * self.stepSizeCoeff  # distance at which the robot is considered at a local min
        self.notMovingLookback = 2  # number of points to look back for local min (avoids oscillation)
        self.angularStart = 30 * np.pi / 180  # angular distance before forward velocity begins

        # Input coordinate params
        self.obstacles = np.empty([0, 2]) # row vectors of obstacles [q1, q2] pos

        # Data storage
        self.calcPath = False  # do we need to calculate a new path

        # Waypoint handlers
        self.waypoints = None
        self.currentWaypoint = None
        self.waypoint_counter = 0

        # Robot initial States
        self.x = 0.0 # x position
        self.y = 0.0 # y position
        self.theta = 0.0 # angular position
        self.linear_vel = 0.3 # linear travel speed
        self.angular_vel = 0.0 # initial angular speed
        self.running = 0.0 # allow robot motion
        self.forward = 0.0 # allow forward motion

        # Global Step Stuff
        self.FrusteratedLocalMinNumber=0

        # PID initial parameters
        self.derivative = 0.0 # derivative term
        self.integral = 0.0 # cummulative integral term
        self.prev_err = 0.0 # last error
        self.int_err = 1.0
        self.prev_time = self.get_clock().now() # last time of running

        # Hardware params
        self.maxVelError = 0.1
        self.maxMotorVel = 2.34
        self.minMotorVel = -2.34
        self.motorVel = 0

        # Sample rate
        self.rate = 10  # 10hz

        # Subscriber data
        self.odomData = None  # stores odometry data
        self.mapData = None # stores map data
        self.goalData = None # stores goal

        # Subscriber topics
        self.odomTopic = '/odom'
        self.mapTopic = '/map'
        self.goalTopic = '/move_base_simple/goal'

        # Data publishing and subscription
        self.create_subscription(Odometry, self.odomTopic, self.odom_callback, self.rate)  # subscribe to the odometry data and when messages are received, self.odom_callback is invoked # NEW
        self.create_subscription(OccupancyGrid, self.mapTopic, self.map_callback, self.rate)  # subscribe to the odometry data and when messages are received, self.odom_callback is invoked # NEW
        self.create_subscription(PoseStamped, self.goalTopic, self.goal_callback, self.rate)  # subscribe to the odometry data and when messages are received, self.odom_callback is invoked # NEW
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.rate)  # publishing velocity data # NEW
        self.err = Float32()

        self.timer = self.create_timer(1 / self.rate, self.run) # run

    def odom_callback(self, msg):  # Initiate odometry data
        self.odomData = msg

    def map_callback(self, msg):  # Initiate odometry data
        now = self.get_clock().now()
        self.mapData = msg
        tempObstacle = self.WhereAreTheObsticles()
        if tempObstacle is not None:
            self.obstacles = tempObstacle

            if self.odomData is not None and self.goalData is not None:
                self.calcPath = True

        # if self.obstacles is not None:
        #     fig = plt.figure(2)
        #     plt.clf()
        #     robotPlot = plt.plot(self.x, self.y, 'r*', label="Robot")
        #     plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ko', label="Obstacles")
        #     plt.title("Obsticle Simulation Simulation")
        #     plt.legend()
        #     plt.pause(0.001)

    def goal_callback(self, msg):  # Initiate odometry data
        if self.odomData is not None:
            self.goalData = [msg.pose.position.x, msg.pose.position.y]
            self.calcPath = True

    def WhereAreTheObsticles(self):
        isRepeat = False
        newEntry = True
        RawMapData = self.mapData
        if RawMapData is None:
            return np.empty((0, 2))
        H = RawMapData.info.height
        W = RawMapData.info.width
        grid = np.array(RawMapData.data, dtype=int).reshape(H, W)

        resolution = RawMapData.info.resolution
        origin = RawMapData.info.origin

        self.stepSizeCoeff = resolution

        q = origin.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        self.actualcord = np.empty((0, 2))
        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                NathanNum = grid[row, col]
                if NathanNum > self.obstacleThreshold:
                    x_cell = col * resolution
                    y_cell = row * resolution
                    x_world = origin.position.x + (x_cell * np.cos(yaw) - y_cell * np.sin(yaw))
                    y_world = origin.position.y + (x_cell * np.sin(yaw) + y_cell * np.cos(yaw))
                    self.actualcord = np.vstack((self.actualcord, [x_world, y_world]))
                    for obs in range(np.shape(self.obstacles)[0]):
                        if self.actualcord[-1,:].tolist() == self.obstacles[obs,:].tolist():
                            isRepeat = True
                    if isRepeat:
                        newEntry = False


        if np.shape(self.obstacles)[0] == np.shape(self.actualcord)[0] or not newEntry:
            self.actualcord = None

        return self.actualcord

    def determineRoute(self):
        now = self.get_clock().now()
        self.calcPath = False
        self.diditrunbefore=False
        currentTargetDist = 100 * self.acceptableTargetError  # initialize distance to enter loop
        self.waypoint_counter = 0
        step = 0 # Reset path starting step

        self.getLocation() # Find out where I am
        robotPoints = np.array([[self.x, self.y]]) # initialize start pos

        fig = plt.figure(1)
        robotPlot = plt.plot(robotPoints[:, 0], robotPoints[:, 1], 'r*', label="Robot")
        plt.plot(self.goalData[0], self.goalData[1], 'bx', label="Goal")
        plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ko', label="Obstacles")
        if self.diditrunbefore==False:
            #plt.legend()
            self.diditrunbefore=True
        #plt.xlim([-1,1])
        plt.title("Path Simulation")
        plt.pause(0.001)

        ## Loop until at goal
        while (abs(currentTargetDist) > self.acceptableTargetError):
            #print("loop")
            Frc = np.zeros(2)
            # Find distance to target
            goalErrorVect = robotPoints[step, :] - self.goalData #Where im at-goal vector so in [x,y]
            currentTargetDist = math.sqrt(np.dot(goalErrorVect, goalErrorVect))  #Euclidean distance to goal from bot

            # Calculate attractive force
            if (currentTargetDist > self.attractiveSwitch): #Far from goal
                Fattract = -self.attractiveForceCoeff * goalErrorVect #PE grows as square of distance (quadratic), force directly proportional to distance
            else: #Near goal
                Fattract = -(self.attractiveForceCoeff * self.attractiveSwitch * goalErrorVect) / currentTargetDist #PE grows as linear to distance, force is constant
                #F = -attractiveForceCoeff * attractiveSwitch * direction_to_goal

            for i in range(np.shape(self.obstacles)[0]):
                # Find distance to obstacle
                obstacleErrorVect = robotPoints[step, :] - self.obstacles[i, :]
                currentObstacleDist = math.sqrt(np.dot(obstacleErrorVect, obstacleErrorVect))

                # Calculate repulsive force
                if (currentObstacleDist <= self.repulsiveSwitch):
                    Frepulsive = self.repulsiveForceCoeff * ((1 / currentObstacleDist) - (1 / self.repulsiveSwitch)) * (1 / currentObstacleDist ** 2) * (obstacleErrorVect / currentObstacleDist)
                else:
                    Frepulsive = np.zeros(2)

                # Calculate cumulative repulsive force
                Frc = Frc + Frepulsive

            # Find resultant force
            Fres = Fattract + Frc

            # Determine next waypoint location
            newPoint = robotPoints[step, :] + self.stepSizeCoeff * (Fres / math.sqrt(np.dot(Fres, Fres)))  # Calculate next point

            if step > self.notMovingLookback:
                stepVect = newPoint - robotPoints[step - (self.notMovingLookback - 1),:]  # Determine step size for local min over multiple iterations to identify oscillation
                self.FrusteratedLocalMinNumber = self.FrusteratedLocalMinNumber + 1
                maxnum=501

                if self.FrusteratedLocalMinNumber <maxnum : #If the og min solution doesnt work, go to a point that alligns with either x or y, then restart the path
                    pass
                elif self.FrusteratedLocalMinNumber >=maxnum :
                    print("Its frusterated")
                    SmallestXToObsticleFromBot=np.min(self.obstacles[:,0]-robotPoints[step,0])
                    SmallestYToObsticleFromBot=np.min(self.obstacles[:,1]-robotPoints[step,1])
                    chosen = 0 if SmallestXToObsticleFromBot > SmallestYToObsticleFromBot else 1
                    #chosen = np.random.randint(0, 2)  # Randomly choose axis: 0 for X-axis, 1 for Y-axis
                    if chosen == 0:
                        print("I choose to go in the X direction \n")
                    elif chosen == 1:
                        print("I choose to go in the Y direction \n")
                    else:
                        print("I don't know what to do \n")
                    if chosen == 0:
                        # Escape along X-axis
                        delta = self.goalData[0] - self.x
                        HonnNum = -3 if delta < 0 else 3
                        escape_pt = np.array([self.x + HonnNum, self.y])
                    else:
                        # Escape along Y-axis
                        delta = self.goalData[1] - self.y
                        HonnNum = -3 if delta < 0 else 3
                        escape_pt = np.array([self.x, self.y + HonnNum])

                    #Add the escape point to the robot path, reset frustration counter
                    robotPoints = np.vstack((robotPoints, escape_pt))
                    self.FrusteratedLocalMinNumber = 0

                    self.waypoints = np.array([escape_pt])  #single‐element waypoint list
                    self.waypoint_counter = 0  #start at element 0
                    self.running = 1.0  #enable movement
                    self.forward = 1.0  #allow forward motion immediately
                    self.calcPath = False  #don’t replan until after this jump
                    return

                # Determine if the robot is trying to move to a spot it was just at (not making progress) and random walk under the assumption of a local min
                if math.sqrt(np.dot(stepVect, stepVect)) < self.notMovingDist and np.dot(Frc, Frc) > 0:
                    print(f"I'm in a local min, im {(float(self.FrusteratedLocalMinNumber/maxnum)*100):.2f}% frusterated \n")
                    FrcH = np.cross([Frc[0], Frc[1], 0], [0, 0, 1])
                    JeffersonValue = FrcH / math.sqrt(np.dot(FrcH, FrcH))
                    newPoint = robotPoints[step, :] + self.stepSizeCoeff * JeffersonValue[0:2]

            # Add new points to the list of points
            robotPoints = np.vstack((robotPoints, newPoint))

            robotPlot[0].set_xdata(newPoint[0])
            robotPlot[0].set_ydata(newPoint[1])
            plt.pause(0.01)

            # Next calc
            step = step + 1

        waypointIncrementer = int(self.waypointParcerSize / self.stepSizeCoeff)

        if np.shape(robotPoints)[0] >= waypointIncrementer:
            self.waypoints = robotPoints[waypointIncrementer::waypointIncrementer]
            self.running = 1.0  # enable movement
            print("Parcing")
        else:
            self.waypoints = None

        # fig = plt.figure(3)
        # robotPlot = plt.plot(robotPoints[:, 0], robotPoints[:, 1], 'r*', label="Robot")
        # plt.plot(self.goalData[0], self.goalData[1], 'bx', label="Goal")
        # plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ko', label="Obstacles")
        # plt.title("Path Simulation")
        # plt.legend()
        # plt.pause(0.001)
        #
        # fig = plt.figure(2)
        # robotPlot = plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'r*', label="Robot")
        # plt.plot(self.goalData[0], self.goalData[1], 'bx', label="Goal")
        # plt.plot(self.obstacles[:, 0], self.obstacles[:, 1], 'ko', label="Obstacles")
        # plt.title("Waypoint Simulation")
        # plt.legend()
        # plt.show()

        print(f"Path planning time {(self.get_clock().now() - now).nanoseconds * 1e-9} \n")

    def getLocation(self):  # get the yaw angle by converting rotations to euler angles
        if self.odomData is not None:
            orientation_q = self.odomData.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            self.theta = yaw
            self.x = self.odomData.pose.pose.position.x
            self.y = self.odomData.pose.pose.position.y


    def saturate(self, input,
                 delta):  # function to saturate the output from the PID controller inside an acceptable range
        if (input > self.maxMotorVel):
            control = self.maxMotorVel
            self.int_err = self.int_err - (self.err * delta)

        elif (input < self.minMotorVel):
            control = self.minMotorVel
            self.int_err = self.int_err - (self.err * delta)
        else:
            control = input
        return control

    def calculate_error(self):
        self.getLocation()

        desiredAngle = np.arctan2(self.currentWaypoint[1] - self.y, self.currentWaypoint[0] - self.x)

        self.err = (desiredAngle - self.theta + (3 * np.pi)) % (2 * np.pi) - np.pi

        # self.err = np.pi / 2 - yaw #tuning code

        print(f"Im at {self.x, self.y}\n")
        print(f"Im heading to {self.currentWaypoint[0], self.currentWaypoint[1]}\n")
        print(f"My final goal is  {self.goalData[0], self.goalData[1]}\n")
        print(f"My error is {self.err}\n")
        print(f"Im desired angle is {desiredAngle}\n")
        return self.err
        #return np.pi/2 - self.theta

    def pid_controller(self):
        current_error = self.calculate_error()  # call the output from the calculate_error() function to use it in the controller
        currentTime = self.get_clock().now()
        dt = (currentTime - self.prev_time).nanoseconds * 1e-9
        self.prev_time = currentTime

        delta = 0

        # take the current area under the area curve and add the new error area
        self.integral = self.integral + (current_error * dt)

        # take the derivative of the rate of change of error
        self.derivative = (current_error - self.prev_err) / dt

        # construct PID
        control = (self.Kp * current_error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

        # ensure we dont overload the motors
        control = self.saturate(control, delta)

        # keep track of last error
        self.prev_err = current_error

        return control

    def run(self):
        self.getLocation()
        angleVel = 0.0
        if self.calcPath:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.determineRoute()

        if self.odomData is not None and self.waypoints is not None:
            if np.shape(self.waypoints)[0] > 0:
                self.currentWaypoint = self.waypoints[self.waypoint_counter, :]
                self.getLocation()
                current_state = np.array([self.x, self.y])
                self.distError = np.linalg.norm(self.currentWaypoint - current_state)  # norm of the difference between two vectors
                angleVel = self.pid_controller()

                if abs(self.calculate_error()) > self.angularStart:
                    self.forward = 0.0 # dont go forward
                else:
                    self.forward = 1.0

                if (self.distError < self.distThreshold):  # the controller runs for a particular waypoint until it reaches a certain threshold
                    self.waypoint_counter += 1
                    if (self.waypoint_counter >= self.waypoints.shape[0]):
                        self.running = 0.0
                        self.waypoints = None
                        print("Completed Path")
                        self.calcPath = True

        twist = Twist()
        twist.linear.x = self.linear_vel * self.running * self.forward
        twist.angular.z =  angleVel * self.running

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = Turtlebot3PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
