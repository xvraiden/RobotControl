import random
import math
import sympy as sy
import matplotlib.pyplot as plt

# Input coordinate params
robotStart = sy.Matrix([[0, 0]]) # row vector [q1, q2] pos
goal = sy.Matrix([[10, 10]]) # row vector [q1, q2] pos 
obstacles = sy.Matrix([[6, 5], [5, 5]]) # row vectors of obstacle [q1, q2] pos

# Tuning coeff
attractiveForceCoeff = 1 # attractive force scalar
repulsiveForceCoeff = 1 # repulsive force scalar
stepSizeCoeff = 0.05 # step size per iteration
randomWalkMag = 0.1 # magnitude of random walk at local min

# Distance thresholds
attractiveSwitch = 1 # distance where attractive force becomes linear
repulsiveSwitch = 1 # distance at which repulsive is negligible
acceptableTargetError = 0.1 # distance at which the robot is close enough to target
notMovingDist = 0.01 # distance at which the robot is considered at a local min
notMovingLookback = 2 # number of points to look back for local min (avoids oscillation)

# Data storage 
robotPoints = robotStart # create matrix to store all robot points and init at start pos
currentTargetDist = 100 * acceptableTargetError # initialize distance to enter loop
step = 0 # init indexer
Frc = sy.zeros(1, 2) # zero matrix to contain cummulative obstacle forces per step


# Display initial state
fig = plt.figure()
robotPlot = plt.plot(robotPoints[0, 0], robotPoints[0, 1], 'r*', label = "Robot")
plt.plot(goal[0], goal[1], 'bx', label = "Goal")
plt.plot(obstacles[:, 0], obstacles[:, 1], 'ko', label = "Obstacles")
plt.title("Path Simulation")
plt.legend()
plt.pause(0.05)

## Loop until at goal
while (abs(currentTargetDist) > acceptableTargetError):
    # Find distance to target
    goalErrorVect = robotPoints[step, :] - goal
    currentTargetDist = math.sqrt(goalErrorVect.dot(goalErrorVect))

    # Calculate attractive force
    if (currentTargetDist > attractiveSwitch):
        Fattract = -attractiveForceCoeff * goalErrorVect
    else:
        Fattract = -(attractiveForceCoeff * attractiveSwitch * goalErrorVect) / currentTargetDist

    for i in range(sy.shape(obstacles)[0]):
        # Find distance to obstacle
        obstacleErrorVect = robotPoints[step, :] - obstacles[i, :]
        currentObstacleDist = math.sqrt(obstacleErrorVect.dot(obstacleErrorVect))

        # Calculate repulsive force
        if (currentObstacleDist <= repulsiveSwitch):
            Frepulsive = repulsiveForceCoeff * ((1 / currentObstacleDist) - (1 / repulsiveSwitch)) * (1 / currentObstacleDist ** 2) * (obstacleErrorVect / currentObstacleDist)
        else:
            Frepulsive = sy.zeros(1, 2)

        # Calculate cummulative repulsive force
        Frc = Frc + Frepulsive

    # Find resultant force
    Fres = Fattract + Frc
    Frc = sy.zeros(1, 2)

    # Determine next waypoint location
    newPoint = robotPoints[step, :] + stepSizeCoeff * (Fres / math.sqrt(Fres.dot(Fres))) # Calculate next point
    stepVect = newPoint - robotPoints[step - (notMovingLookback - 1), :] # Determine step size for local min over 2 iterations to identify oscillation

    # Determine if the robot is trying to move to a spot it was just at (not making progress) and random walk under the assumption of a local min
    if (math.sqrt(stepVect.dot(stepVect)) < notMovingDist):
        print("local min")
        newPoint = robotPoints[step, :] + sy.Matrix([[random.uniform(-randomWalkMag, randomWalkMag), random.uniform(-randomWalkMag, randomWalkMag)]]) # random walk

    # Add new points to the list of points
    robotPoints = robotPoints.row_insert(sy.shape(robotPoints)[0], newPoint)

    # Display new position on plot
    robotPlot[0].set_xdata([robotPoints[step, 0]])
    robotPlot[0].set_ydata([robotPoints[step, 1]])
    plt.pause(0.05)

    # Next calc
    step = step + 1