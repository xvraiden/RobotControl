import sympy as sy
import math
import numpy as np

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

## Setup ##
# Generate DH table in inches and radians
dhVals = sy.Matrix([[0, -sy.pi / 2, sy.Symbol("d1"), 0], [0, sy.pi / 2, sy.Symbol("d2"), -sy.pi / 2], [0, 0, sy.Symbol("d3"),  0]]) #cliff
#dhVals = sy.Matrix([[0, sy.pi / 2, sy.Symbol("d1"), -sy.pi / 2], [0, sy.pi / 2, sy.Symbol("d2"), sy.pi / 2], [0, 0, sy.Symbol("d3"),  0]]) #nathan
#dhVals = sy.Matrix([[0, -sy.pi / 2, sy.Symbol("d1"), 0], [0, sy.pi / 2, sy.Symbol("d2"), -sy.pi / 2], [0, 0, sy.Symbol("d3"),  0]]) #brian


## Part i ##
# Find symbolic transformation matrix for successive frame
T00 = sy.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T01 = DHSymbolic(dhVals[0, 0], dhVals[0, 1], dhVals[0, 2], dhVals[0, 3])
T12 = DHSymbolic(dhVals[1, 0], dhVals[1, 1], dhVals[1, 2], dhVals[1, 3])
T23 = DHSymbolic(dhVals[2, 0], dhVals[2, 1], dhVals[2, 2], dhVals[2, 3])

# Find symbolic transformation matrix for 0th frame
T02 = sy.simplify(T01 * T12)
T03 = sy.simplify(T02 * T23)

# Display simplified successive frame and T06 matrices
print("T01:")
sy.pprint(T01)

print("\nT12:")
sy.pprint(T12)

print("\nT23:")
sy.pprint(T23)

print("\nT03:")
sy.pprint(T03)

## Begin Jacobian ##
# Determine each joint jacobian
# Joint 1 (prismatic)
J1 = sy.Matrix([T00[0:3, 2], sy.zeros(3, 1)])

# Joint 2 (prismatic)
J2 = sy.Matrix([T01[0:3, 2], sy.zeros(3, 1)])

# Joint 3 (prismatic)
J3 = sy.Matrix([T02[0:3, 2], sy.zeros(3, 1)])


# Combine joint columns
Jsym = sy.Matrix([[J1.T], [J2.T], [J3.T]]).T

print("Jacobian:")
sy.pprint(Jsym)