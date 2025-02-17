import numpy as np
import sympy as sy
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

## Setup Symbols ##
# Create generic DH symbols
alphaSy = sy.Symbol("α")
thetaSy = sy.Symbol("Θ")
aSy = sy.Symbol("a")
dSy = sy.Symbol("d")

##  Setup matrices
# Create DH table with unique symbols and numbers
dhVals = sy.Matrix([[4, 0, 0, sy.Symbol("Θ1")], [3, 0, 0, sy.Symbol("Θ2")], [2, 0, 0, sy.Symbol("Θ3")]])

# Determine the generic transformation matrix
TGen = DHSymbolic(aSy, alphaSy, dSy, thetaSy)

# Determine the transformation matrix for each reference frame by swapping local symbols into generic matrix
T01 = TGen.subs({aSy:dhVals[0, 0], alphaSy:dhVals[0, 1], dSy:dhVals[0,2], thetaSy:dhVals[0,3]})
T12 = TGen.subs({aSy:dhVals[1, 0], alphaSy:dhVals[1, 1], dSy:dhVals[1,2], thetaSy:dhVals[1,3]})
T23 = TGen.subs({aSy:dhVals[2, 0], alphaSy:dhVals[2, 1], dSy:dhVals[2,2], thetaSy:dhVals[2,3]})

## Begin computing link transformations
# Determine transformation for T02, T03 by current multiplication
T02 = T01 * T12
T03 = T02 * T23

# Display T03 for comparison
sy.pprint(sy.simplify(T03))

# Determine end effector location for case 1
case1C01 = T01.subs({sy.Symbol("Θ1"):math.radians(0), sy.Symbol("Θ2"):math.radians(0), sy.Symbol("Θ3"):math.radians(0)})
case1C02 = T02.subs({sy.Symbol("Θ1"):math.radians(0), sy.Symbol("Θ2"):math.radians(0), sy.Symbol("Θ3"):math.radians(0)})
case1C03 = T03.subs({sy.Symbol("Θ1"):math.radians(0), sy.Symbol("Θ2"):math.radians(0), sy.Symbol("Θ3"):math.radians(0)})

# Determine the end effector location for case 2
case2C01 = T01.subs({sy.Symbol("Θ1"):math.radians(10), sy.Symbol("Θ2"):math.radians(20), sy.Symbol("Θ3"):math.radians(30)})
case2C02 = T02.subs({sy.Symbol("Θ1"):math.radians(10), sy.Symbol("Θ2"):math.radians(20), sy.Symbol("Θ3"):math.radians(30)})
case2C03 = T03.subs({sy.Symbol("Θ1"):math.radians(10), sy.Symbol("Θ2"):math.radians(20), sy.Symbol("Θ3"):math.radians(30)})

# Determine the end effector location for case 3
case3C01 = T01.subs({sy.Symbol("Θ1"):math.radians(90), sy.Symbol("Θ2"):math.radians(90), sy.Symbol("Θ3"):math.radians(90)})
case3C02 = T02.subs({sy.Symbol("Θ1"):math.radians(90), sy.Symbol("Θ2"):math.radians(90), sy.Symbol("Θ3"):math.radians(90)})
case3C03 = T03.subs({sy.Symbol("Θ1"):math.radians(90), sy.Symbol("Θ2"):math.radians(90), sy.Symbol("Θ3"):math.radians(90)})


## Plot arm configuration ##
# Determine the location x, y, z of each link end
# Case 1
case1C01Pos = case1C01[:, 3]
case1C02Pos = case1C02[:, 3]
case1C03Pos = case1C03[:, 3]

# Case 2
case2C01Pos = case2C01[:, 3]
case2C02Pos = case2C02[:, 3]
case2C03Pos = case2C03[:, 3]

# Case 3
case3C01Pos = case3C01[:, 3]
case3C02Pos = case3C02[:, 3]
case3C03Pos = case3C03[:, 3]


# Plot each link for case 1
fig1 = plt.figure(1)
plt.plot([0, case1C01Pos[0]], [0, case1C01Pos[1]], '-r', [case1C01Pos[0], case1C02Pos[0]], [case1C01Pos[1], case1C02Pos[1]], '-g', [case1C02Pos[0], case1C03Pos[0]], [case1C02Pos[1], case1C03Pos[1]], '-b' )
plt.legend(["Link 1", "Link 2", "Link 3"])
plt.title("Case 1 Arm Configuration")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")


# Plot each link for case 2
fig2 = plt.figure(2)
plt.plot([0, case2C01Pos[0]], [0, case2C01Pos[1]], '-r', [case2C01Pos[0], case2C02Pos[0]], [case2C01Pos[1], case2C02Pos[1]], '-g', [case2C02Pos[0], case2C03Pos[0]], [case2C02Pos[1], case2C03Pos[1]], '-b' )
plt.legend(["Link 1", "Link 2", "Link 3"])
plt.title("Case 2 Arm Configuration")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")

# Plot each link for case 3
fig3 = plt.figure(3)
plt.plot([0, case3C01Pos[0]], [0, case3C01Pos[1]], '-r', [case3C01Pos[0], case3C02Pos[0]], [case3C01Pos[1], case3C02Pos[1]], '-g', [case3C02Pos[0], case3C03Pos[0]], [case3C02Pos[1], case3C03Pos[1]], '-b' )
plt.legend(["Link 1", "Link 2", "Link 3"])
plt.title("Case 3 Arm Configuration")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")


## Animate arm motion ##
# Create time array from 0 to 5 by 0.01 seconds
time = np.arange(0, 5, 0.01)

# Create storage matrix for positions at each time step
case1D01Pos = sy.zeros(4, 3 * len(time))
case1D02Pos = sy.zeros(4, 3 * len(time))
case1D03Pos = sy.zeros(4, 3 * len(time))

case2D01Pos = sy.Matrix()
case2D02Pos = sy.Matrix()
case2D03Pos = sy.Matrix()

# Determine forward kinematics for each time step
for index, t in enumerate(time):
    # calculate theta for each time step
    theta = 2 * math.pi * math.sin(t)

    for i in range(3):
        # Generate angles for calculations
        thetas = [0, 0, 0]
        thetas[i] = theta

        # Determine end effector location for case 1 by moving each join independently
        case1D01 = T01.subs({sy.Symbol("Θ1"):thetas[0], sy.Symbol("Θ2"):thetas[1], sy.Symbol("Θ3"):thetas[2]})
        case1D02 = T02.subs({sy.Symbol("Θ1"):thetas[0], sy.Symbol("Θ2"):thetas[1], sy.Symbol("Θ3"):thetas[2]})
        case1D03 = T03.subs({sy.Symbol("Θ1"):thetas[0], sy.Symbol("Θ2"):thetas[1], sy.Symbol("Θ3"):thetas[2]})

        # Append position to array for plotting (first set of time is q1, second set q2, third set q3). Array is 4x(len(time) * 3)
        case1D01Pos[:, index + (i * len(time))] = case1D01[:, 3]
        case1D02Pos[:, index + (i * len(time))] = case1D02[:, 3]
        case1D03Pos[:, index + (i * len(time))] = case1D03[:, 3]

    # Determine the end effector location for case 2
    case2D01 = T01.subs({sy.Symbol("Θ1"):theta, sy.Symbol("Θ2"):theta, sy.Symbol("Θ3"):theta})
    case2D02 = T02.subs({sy.Symbol("Θ1"):theta, sy.Symbol("Θ2"):theta, sy.Symbol("Θ3"):theta})
    case2D03 = T03.subs({sy.Symbol("Θ1"):theta, sy.Symbol("Θ2"):theta, sy.Symbol("Θ3"):theta})

    # Append position to array for plotting
    case2D01Pos = case2D01Pos.col_insert(sy.shape(case2D01Pos)[1], case2D01[:, 3])
    case2D02Pos = case2D02Pos.col_insert(sy.shape(case2D02Pos)[1], case2D02[:, 3])
    case2D03Pos = case2D03Pos.col_insert(sy.shape(case2D03Pos)[1], case2D03[:, 3])


# Plot the first time step
fig4 = plt.figure(4)
case1DPlot = plt.plot([0, case1D01Pos[0, 0]], [0, case1D01Pos[1, 0]], "-r", [case1D01Pos[0, 0], case1D02Pos[0, 0]], [case1D01Pos[1, 0], case1D02Pos[1, 0]], "-g", [case1D02Pos[0, 0], case1D03Pos[0, 0]], [case1D02Pos[1, 0], case1D03Pos[1, 0]], "-b")
plt.legend(["Link 1", "Link 2", "Link 3"])
plt.title("Arm Simulation Case 1")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")
plt.xlim(left = -9.25, right = 9.25)
plt.ylim(bottom = -9.25, top = 9.25)


# Plot the first time step
fig5 = plt.figure(5)
case2DPlot = plt.plot([0, case2D01Pos[0, 0]], [0, case2D01Pos[1, 0]], "-r", [case2D01Pos[0, 0], case2D02Pos[0, 0]], [case2D01Pos[1, 0], case2D02Pos[1, 0]], "-g", [case2D02Pos[0, 0], case2D03Pos[0, 0]], [case2D02Pos[1, 0], case2D03Pos[1, 0]], "-b")
plt.legend(["Link 1", "Link 2", "Link 3"])
plt.title("Arm Simulation Case 2")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")
plt.xlim(left = -9.25, right = 9.25)
plt.ylim(bottom = -9.25, top = 9.25)

def animate(frame, data01, data02, data03, linePlot):
    # Update the data for each of the three links
    linePlot[0].set_xdata([0, data01[0, frame]])
    linePlot[0].set_ydata([0, data01[1, frame]])
    linePlot[1].set_xdata([data01[0, frame], data02[0, frame]])
    linePlot[1].set_ydata([data01[1, frame], data02[1, frame]])
    linePlot[2].set_xdata([data02[0, frame], data03[0, frame]])
    linePlot[2].set_ydata([data02[1, frame], data03[1, frame]])
    return linePlot


# Animate remaining time steps for cases
ani1 = FuncAnimation(fig4, animate, frames = sy.shape(case1D01Pos)[1], fargs = (case1D01Pos, case1D02Pos, case1D03Pos, case1DPlot), interval = 10)
ani2 = FuncAnimation(fig5, animate, frames = sy.shape(case2D01Pos)[1], fargs = (case2D01Pos, case2D02Pos, case2D03Pos, case2DPlot), interval = 10)
plt.show()

## Generate Gif ##
#writer = animation.PillowWriter(fps=60, metadata=dict(artist='Me'), bitrate=1800)
#ani1.save('case1.gif', writer=writer)
#ani2.save('case2.gif', writer=writer)