import math
import sympy as sy
import numpy as np

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

## Setup Values ##
# Desired robot pose
T04 = sy.Matrix([[0, 1, 0, 0.3536], [1, 0, 0, 0.8536], [0, 0, -1, -1], [0, 0, 0, 1]])

# Given robot link properties from DH table
a1 = 0.5
a2 = 0.5
d4 = 0.5

DHParams = sy.Matrix([[a1, 0, 0, sy.Symbol("Θ1")], [a2, sy.pi, 0, sy.Symbol("Θ2")], [0, 0, sy.Symbol("d3"), 0], [0, 0, d4, sy.Symbol("Θ4")]])

# Break T05 into rotation and translation matrices
R04 = T04[0:3, 0:3]
O04 = T04[0:3, 3]

## Begin inverse kinematics ##
# Determine alpha
alpha = sy.atan2(R04[0, 1], R04[0, 0])

# Determine cosine of theta 2
c2 = (O04[0] **2 + O04[1] **2 - a1 **2 - a2 ** 2) / (2 * a1 * a2)

# Determine theta 2
theta2 = sy.atan2(math.sqrt(1 - c2 ** 2), c2)

# Determine theta 1
theta1 = sy.atan2(O04[1], O04[0]) - sy.atan2(a2 * sy.sin(theta2), a1 + a2 * c2)

# Determine theta 4
theta4 = theta1 + theta2 - alpha

# Determine d3
d3 = -O04[2] - d4

# Display calculated join positions
print(f"Theta 1:\n{math.degrees(theta1)} degrees\n")
print(f"Theta 2:\n{math.degrees(theta2)} degrees\n")
print(f"Theta 4:\n{math.degrees(theta4)} degrees\n")
print(f"d3:\n{d3} meters down\n")

# Plug positions into DH matrix to conduct forward kinematics
DHParams = DHParams.subs({sy.Symbol("Θ1"):theta1, sy.Symbol("Θ2"):theta2, sy.Symbol("d3"):d3, sy.Symbol("Θ4"):theta4})

# Solve forward kinematics for verification
T01 = DHSymbolic(DHParams[0, 0], DHParams[0, 1], DHParams[0, 2], DHParams[0, 3])
T12 = DHSymbolic(DHParams[1, 0], DHParams[1, 1], DHParams[1, 2], DHParams[1, 3])
T23 = DHSymbolic(DHParams[2, 0], DHParams[2, 1], DHParams[2, 2], DHParams[2, 3])
T34 = DHSymbolic(DHParams[3, 0], DHParams[3, 1], DHParams[3, 2], DHParams[3, 3])

T04 = T01 * T12 * T23 * T34

sy.pprint(T04)