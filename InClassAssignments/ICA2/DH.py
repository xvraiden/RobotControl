import numpy as np
import sympy as sy
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def DenavitHartenburgGenerator(a, alpha, d, theta):
    row1 = np.array([math.cos(np.deg2rad(theta)), -math.sin(np.deg2rad(theta)) * math.cos(np.deg2rad(alpha)), math.sin(np.deg2rad(theta)) * math.sin(np.deg2rad(alpha)), a * math.cos(np.deg2rad(theta))])
    row2 = np.array([math.sin(np.deg2rad(theta)), math.cos(np.deg2rad(theta)) * math.cos(np.deg2rad(alpha)), -math.cos(np.deg2rad(theta)) * math.sin(np.deg2rad(alpha)), a * math.sin(np.deg2rad(theta))])
    row3 = np.array([0, math.sin(np.deg2rad(alpha)), math.cos(np.deg2rad(alpha)), d])
    row4 = np.array([0, 0, 0, 1])
    return np.array([row1, row2, row3, row4])

def PostMultiplier(originalFrame, newFrame):
    return np.matmul(originalFrame, newFrame)

# Generate DH matrix for origin 2 WRT origin 1
T12 = DenavitHartenburgGenerator(0.5, 180, 0, 90)

# Generate DH matrix for origin 3 WRT origin 2
T23 = DenavitHartenburgGenerator(0, 0, 0.5, 0)

# Generate DH matrix for origin 4 WRT origin 3
T34 = DenavitHartenburgGenerator(0, 0, 0.5, 0)

# Generate point storage array
points01 = np.zeros([3, 1])
points02 = np.zeros([3, 1])
points03 = np.zeros([3, 1])
points04 = np.zeros([3, 1])

for theta1 in range(0, 360, 1):
    # Generate DH matrix for origin 1 WRT origin 0
    T01 = DenavitHartenburgGenerator(0.5, 0, 0, theta1)

    # Perform Post-Multipication to get the end effector (origin 1) in terms of the base (origin 0)
    DH01 = T01

    # Perform Post-Multipication to get the end effector (origin 2) in terms of the base (origin 0)
    DH02 = PostMultiplier(DH01, T12)

    # Perform Post-Multipication to get the end effector (origin 3) in terms of the base (origin 0)
    DH03 = PostMultiplier(DH02, T23)

    # Perform Post-Multipication to get the end effector (origin 4) in terms of the base (origin 0)
    DH04 = PostMultiplier(DH03, T34)    

    # Append points of each link to their repsective array
    points01 = np.append(points01, np.array([[DH01[0, 3]], [DH01[1, 3]], [DH01[2, 3]]]), 1)
    points02 = np.append(points02, np.array([[DH02[0, 3]], [DH02[1, 3]], [DH02[2, 3]]]), 1)
    points03 = np.append(points03, np.array([[DH03[0, 3]], [DH03[1, 3]], [DH03[2, 3]]]), 1)
    points04 = np.append(points04, np.array([[DH04[0, 3]], [DH04[1, 3]], [DH04[2, 3]]]), 1)

plotLines = []

figure1 = plt.figure()
ax = plt.axes(projection = '3d')
plotLines.append(ax.plot3D(points01[0, 1], points01[1, 1], points01[2, 1]))
plotLines.append(ax.plot3D(points02[0, 1], points02[1, 1], points02[2, 1]))
plotLines.append(ax.plot3D(points03[0, 1], points03[1, 1], points03[2, 1]))
plotLines.append(ax.plot3D(points04[0, 1], points04[1, 1], points04[2, 1]))
ax.set_title("Link End Position Under Base Rotation")
ax.set_xlabel("X Coordinate (m)")
ax.set_ylabel("Y Coordinate (m)")
ax.set_zlabel("Z Coordinate (m)")
ax.set_xlim3d(left = -1.25, right = 1.25)
ax.set_ylim3d(bottom = -1.25, top = 1.25)
ax.set_zlim3d(bottom = -1.25, top = 1.25)
ax.legend(["Link 1", "Link 2", "Link 3", "Link 4"])

print(plotLines)

def animate(frame):    
    # Update the data for each of the four links
    plotLines[0][0].set_data_3d(points01[0, 1:frame], points01[1, 1:frame], points01[2, 1:frame])
    plotLines[1][0].set_data_3d(points02[0, 1:frame], points02[1, 1:frame], points02[2, 1:frame])
    plotLines[2][0].set_data_3d(points03[0, 1:frame], points03[1, 1:frame], points03[2, 1:frame])
    plotLines[3][0].set_data_3d(points04[0, 1:frame], points04[1, 1:frame], points04[2, 1:frame])
    return plotLines

ani = FuncAnimation(figure1, animate, frames = np.shape(points01)[1], interval = 1)
plt.show()