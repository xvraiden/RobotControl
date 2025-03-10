import sympy as sy
import math
import numpy as np

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

def Euler(RotationMatrix):
    # Find phi(rotation 1) using Euler
    phiEuler = np.rad2deg(math.atan2(RotationMatrix[1, 2], RotationMatrix[0, 2]))

    # Find theta(rotation 2) using Euler
    thetaEuler = np.rad2deg(math.atan2(math.sqrt(1 - RotationMatrix[2, 2]**2), RotationMatrix[2, 2]))

    # Find psi(rotation 3) using Euler
    psiEuler = np.rad2deg(math.atan2(RotationMatrix[2, 1], -RotationMatrix[2, 0]))

    return [phiEuler, thetaEuler, psiEuler]

## Setup ##
# Generate DH table in inches and radians
dhVals = sy.Matrix([[0, sy.pi / 2, sy.Symbol("a1"), sy.Symbol("Θ1") + sy.pi / 2], [sy.Symbol("a2"), 0, 0, sy.Symbol("Θ2")], [sy.Symbol("a3"), 0, 0,  sy.Symbol("Θ3")]])

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

print("\nT02:")
sy.pprint(T02)

print("\nT03:")
sy.pprint(T03)


## Begin Jacobian ##
# Determine each joint jacobian
# Joint 1 (revolute)
J1 = sy.Matrix([T00[0:3, 2].cross(T03[0:3, 3] - T00[0:3, 3]), T00[0:3, 2]])

# Joint 2 (revolute)
J2 = sy.Matrix([T01[0:3, 2].cross(T03[0:3, 3] - T01[0:3, 3]), T01[0:3, 2]])

# Joint 3 (revolute)
J3 = sy.Matrix([T02[0:3, 2].cross(T03[0:3, 3] - T02[0:3, 3]), T02[0:3, 2]])


# Combine joint columns
Jsym = sy.Matrix([[J1.T], [J2.T], [J3.T]]).T

print("Jacobian:")
sy.pprint(sy.simplify(Jsym))