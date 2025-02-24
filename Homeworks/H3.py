import sympy as sy
import math

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

## Setup ##
# Generate DH table in inches and radians
dhVals = sy.Matrix([[0, sy.pi / 2, 13, sy.Symbol("Θ1")], [8, 0, 0, sy.Symbol("Θ2")], [8, sy.pi / 2, 0, sy.Symbol("Θ3")], [0, -sy.pi / 2, 0, sy.Symbol("Θ4")], [0, sy.pi / 2, 0, sy.Symbol("Θ5")], [0, 0, 5, sy.Symbol("Θ6")]])

## Part i ##
# Find symbolic transformation matrix for successive frame
T01 = DHSymbolic(dhVals[0, 0], dhVals[0, 1], dhVals[0, 2], dhVals[0, 3])
T12 = DHSymbolic(dhVals[1, 0], dhVals[1, 1], dhVals[1, 2], dhVals[1, 3])
T23 = DHSymbolic(dhVals[2, 0], dhVals[2, 1], dhVals[2, 2], dhVals[2, 3])
T34 = DHSymbolic(dhVals[3, 0], dhVals[3, 1], dhVals[3, 2], dhVals[3, 3])
T45 = DHSymbolic(dhVals[4, 0], dhVals[4, 1], dhVals[4, 2], dhVals[4, 3])
T56 = DHSymbolic(dhVals[5, 0], dhVals[5, 1], dhVals[5, 2], dhVals[5, 3])

# Find symbolic transformation matrix for 0th frame
T02 = sy.simplify(T01 * T12)
T03 = sy.simplify(T02 * T23)
T04 = sy.simplify(T03 * T34)
T05 = sy.simplify(T04 * T45)
T06 = sy.simplify(T05 * T56)

# Display simplified successive frame and T06 matrices
print("T01:")
sy.pprint(T01)

print("\nT12:")
sy.pprint(T12)

print("\nT23:")
sy.pprint(T23)

print("\nT34:")
sy.pprint(T34)

print("\nT45:")
sy.pprint(T45)

print("\nT56:")
sy.pprint(T56)

print("\nT06:")
sy.pprint(T06)
#sy.pprint(T06[:,0])
#sy.pprint(T06[:,1])
#sy.pprint(T06[:,2])
#sy.pprint(T06[:,3])

## Part ii ##
# Generate transformation matrix for inverse kinematics
inverseEnd = sy.Matrix([[0.25, 0.4571, 0.8536, 8.2678], [-0.9571, 0.25, 0.1464, 4.7322], [-0.1464, -0.8536, 0.5, 29.1569], [0, 0, 0, 1]])

# Split transformation into rotation and translation
R06 = inverseEnd[0:3, 0:3]
O06 = inverseEnd[0:3, 3]

# Find wrist center Oc
wristCenter = O06 - dhVals[5, 2] * R06 * sy.Matrix([[0], [0], [1]])

# Determine theta1
theta1 = sy.atan2(wristCenter[1], wristCenter[0])

# Determine theta3
D = (wristCenter[0] **2 + wristCenter[1] **2 + (wristCenter[2] - dhVals[0, 2]) ** 2 - dhVals[1, 0] ** 2 - dhVals[2, 0] ** 2) / (2 * dhVals[1, 0] * dhVals[2, 0])
theta3 = sy.atan2(math.sqrt(1 - D ** 2), D)

# Determine theta2
theta2 = sy.atan2(wristCenter[2] - dhVals[0, 2], math.sqrt(wristCenter[0] **2 + wristCenter[1] **2)) - sy.atan2(dhVals[2, 0] * sy.sin(theta3), dhVals[1, 0] + dhVals[2, 0] * sy.cos(theta3))

# Determine R03
R03 = T03.subs({sy.Symbol("Θ1"):theta1, sy.Symbol("Θ2"):theta2, sy.Symbol("Θ3"):theta3})[0:3, 0:3]

# Determine R36
R36 = R03.T * R06

# Determine theta4
theta4 = sy.atan2(R36[1, 2], R36[0, 2])

# Determine theta5
theta5 = sy.atan2(math.sqrt(1 - R36[2, 2] ** 2), R36[2, 2])

# Determine theta6
theta6 = sy.atan2(R36[2, 1], -R36[2, 0])

# Display join angles in degrees
print(f"Θ1: {round(math.degrees(theta1), 2)} degrees\n")
print(f"Θ2: {round(math.degrees(theta2), 2)} degrees\n")
print(f"Θ3: {round(math.degrees(theta3), 2)} degrees\n")
print(f"Θ4: {round(math.degrees(theta4), 2)} degrees\n")
print(f"Θ5: {round(math.degrees(theta5), 2)} degrees\n")
print(f"Θ6: {round(math.degrees(theta6), 2)} degrees\n")

# Validate solution by performing forward kinematics
Tcheck = T06.subs({sy.Symbol("Θ1"):theta1, sy.Symbol("Θ2"):theta2, sy.Symbol("Θ3"):theta3, sy.Symbol("Θ4"):theta4, sy.Symbol("Θ5"):theta5, sy.Symbol("Θ6"):theta6})
print("Transformation matrix 06:")

TForward = sy.zeros(4, 4)

for rowI in range(4):
    for colI in range (4):
        TForward[rowI, colI] = round(Tcheck[rowI, colI], 4)

sy.pprint(TForward)