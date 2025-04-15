import matplotlib.pyplot as plt
import numpy as np
import random
import time
from numpy.linalg import norm as norm
from numpy import sin as sin
from numpy import cos as cos
from numpy import pi as pi
from numpy.linalg import inv as inv
from matplotlib.animation import FuncAnimation

# Function to calculate the coefficients of cubic
def getCubicCoefficients(q0,qf,v0,vf,t0,tf):
    M = np.array([[1,t0,t0**2,t0**3],[0,1,2*t0,3*t0**2],[1,tf,tf**2,tf**3],[0,1,2*tf,3*tf**2]])
    b = np.array([q0,v0,qf,vf])
    coefficients = inv(M).dot(b)
    return coefficients.tolist()

# Inputs
joint1Loc = np.array([0,0]) # Anchor point at center
joint2Loc = np.array([0,-0.2]) # Initial Joint Location
robotLoc = np.array([0, -0.4]) # Initial End Effector Position
goal1Loc = np.array([0, 0.4]) # Goal 1 Position
goal2Loc = np.array([0, 0.2]) # Goal 2 Position
obstacleLoc = np.array([[0.22, -0.22], [-0.22,-0.22], [0,0]]) # Obstacle Locations
goalThresholdDis = .0015 # Distance from goal that counts as a success

# Waypoints for Cubics
theta1Waypoints = [
    [-90,-83.3,0,0,0,1],
    [-83.3,-14.7,0,0,1,2],
    [-14.7,82.9,0,0,2,3],
    [82.9,115,0,0,3,4],
    [115,90,0,0,4,5]
]

theta2Waypoints = [
    [0,-18.3,0,0,0,1],
    [-18.3,-160,0,0,1,2],
    [-160,-153.6,0,0,2,3],
    [-153.6,-83.7,0,0,3,4],
    [-83.7,0,0,0,4,5]
]

'''
# Original Angles that may be wrong
theta2Coefficients = [
    [-90,-135,0,0,0,1],
    [-135,-174.8,0,0,1,2],
    [-174.8,-86.9,0,0,2,3],
    [-86.9,-32.2,0,0,3,4],
    [-32.2,86,0,0,4,5]
]
'''
   
theta1FullTrajectory = [] # Theta 1's joint angles with times to be followed
theta2FullTrajectory = [] # Theta 2's joint angles with times to be followed
joint2FullPostition = [] # Joint 2's position to be followed
robotLocFullPosition = [] # End effector's position to be followed
allTime = [] # Every time where there is a angle entry
stepTime = [] # Step between each time
samplesPerSegment = 100

for i in range(len(theta1Waypoints)):
    t0 = i # Initial time for this loop
    tf = i+1 # Final time for this loop
    tSegment = np.linspace(t0,tf,samplesPerSegment) # Cuts time segment into little pieces for each position calculation
    tStep = np.ones(samplesPerSegment)*(tf-t0)/samplesPerSegment

    a0J1, a1J1, a2J1, a3J1 = getCubicCoefficients(theta1Waypoints[i][0],theta1Waypoints[i][1],theta1Waypoints[i][2],theta1Waypoints[i][3],theta1Waypoints[i][4],theta1Waypoints[i][5])
    a0J2, a1J2, a2J2, a3J2 = getCubicCoefficients(theta2Waypoints[i][0],theta2Waypoints[i][1],theta2Waypoints[i][2],theta2Waypoints[i][3],theta2Waypoints[i][4],theta2Waypoints[i][5])
    theta1Segment = a0J1 + a1J1*tSegment + a2J1*tSegment**2 + a3J1*tSegment**3
    theta2Segment = a0J2 + a1J2*tSegment + a2J2*tSegment**2 + a3J2*tSegment**3
    theta1FullTrajectory.append(theta1Segment)
    theta2FullTrajectory.append(theta2Segment)
    allTime.append(tSegment)
    stepTime.append(tStep)

theta1FullTrajectory = np.concatenate(theta1FullTrajectory)
theta2FullTrajectory = np.concatenate(theta2FullTrajectory)
allTime = np.round(np.concatenate(allTime), 2)
stepTime = np.concatenate(stepTime)

mainFigure = plt.figure()

plt.title("Workspace View")
plt.xlabel("X Distance (m)")
plt.ylabel("Y Distance (m)")

plt.ion()
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
plt.scatter(goal1Loc[0], goal1Loc[1])
timeText = plt.text(-0.4,0.4,f"Time = {0} seconds")

robotPlotCurrent, = plt.plot(robotLoc[0], robotLoc[1], 'bo', markersize=5)
joint2PlotCurrent, = plt.plot(joint2Loc[0], joint2Loc[1], 'go', markersize=5)
joint1PlotCurrent, = plt.plot(joint1Loc[0], joint1Loc[1], 'yo', markersize=5)
link1LineCurrent, = plt.plot([joint1Loc[0], joint2Loc[0]], [joint1Loc[1], joint2Loc[1]], 'g-', linewidth=2)
link2LineCurrent, = plt.plot([joint2Loc[0], robotLoc[0]], [joint2Loc[1], robotLoc[1]], 'b-', linewidth=2)
link1Length = 0.2
link2Length = 0.2
for i in range(obstacleLoc.shape[0]):
    plt.scatter(obstacleLoc[i, 0], obstacleLoc[i, 1], c='c', marker='o')

plt.pause(0.0001) # Freezes the figure right before it starts the loop so you have something to gaze at while it does the math for a while
startTime = time.time()

# Loop
robotPath = [] # Stores robot's location for the path picture at the end and as reference for joint graphs
joint2LocHistory = [] # Stores joint 2's location for the for the joint graphs
joint1AngleHistory = []
joint2AngleHistory = []
timeHistory = []
counter = 0

joint1Angle = theta1FullTrajectory*(pi/180)
joint2Angle = theta2FullTrajectory*(pi/180)
for i in range(len(allTime)):
    joint2FullPostition.append(link1Length*np.array([cos(joint1Angle[i]), sin(joint1Angle[i])]))
    robotLocFullPosition.append(joint2FullPostition[i]+link2Length*np.array([np.cos(joint1Angle[i] + joint2Angle[i]), np.sin(joint1Angle[i] + joint2Angle[i])]))


for i in range(len(allTime)):
    for pastLoc in robotPath[-1:]:
        plt.scatter(pastLoc[0],pastLoc[1], c='blue', s=1, alpha=0.3)
    '''
    print("Link1 length: " + str(norm(joint2Loc[i]-joint1Loc[i])) + ", Link2 length: " + str(norm(robotLoc[i]-joint2Loc[i])))
    robotLocWRTjoint2 = robotLoc[i]-joint2Loc[i] # End effector location with respect to joint 2 location for angle calculation
    joint1Angle = (np.arctan2(joint2Loc[1] ,joint2Loc[0]))*(180/pi) # Calculates Joint 1 angle for future graph (deg). The theta = 0 position is when the arm is to the right.
    joint2Angle = (np.arctan2(robotLocWRTjoint2[1], robotLocWRTjoint2[0]))*(180/pi) # Calculates Joint 2 angle for future graph (deg). The theta = 0 position is when the arm is to the right.
    currentTime = time.time() - startTime
    
    timeHistory.append(currentTime)
    robotPath.append(robotLoc.copy())
    joint2LocHistory.append(joint2Loc.copy())
    joint1AngleHistory.append(joint1Angle) # Stores the value of the joint 1 angle for future graph 
    joint2AngleHistory.append(joint2Angle)
    '''

    robotPlotCurrent.set_data([robotLocFullPosition[i][0]], [robotLocFullPosition[i][1]])
    joint2PlotCurrent.set_data([joint2FullPostition[i][0]], [joint2FullPostition[i][1]])
    joint1PlotCurrent.set_data([joint1Loc[0]], [joint1Loc[1]])
    link1LineCurrent.set_data([joint1Loc[0], joint2FullPostition[i][0]], [joint1Loc[1], joint2FullPostition[i][1]])
    link2LineCurrent.set_data([joint2FullPostition[i][0], robotLocFullPosition[i][0]], [joint2FullPostition[i][1], robotLocFullPosition[i][1]])
    timeText.set_text(f"Time = {allTime[i]} seconds")
    plt.pause(stepTime[i])
    counter += 1

endTime = time.time()
totalTime = endTime - startTime

if (norm(robotLoc - goal1Loc) <= goalThresholdDis):
    print("The robot made it to the goal! Sucess!")
else:
    print("Failure :(")

joint1Graph = plt.figure()
plt.plot(allTime, theta1FullTrajectory)
plt.title("Joint 1 vs Time")
plt.ylabel("Angle (deg)")
plt.xlabel("Time (sec)")

joint2Graph = plt.figure()
plt.plot(allTime, theta2FullTrajectory)
plt.title("Joint 2 vs Time")
plt.ylabel("Angle (deg)")
plt.xlabel("Time (sec)")

plt.ioff()
plt.show()
