import sympy as sy

def DHSymbolic(a,α,d,Θ):
    row1 = sy.Matrix([sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)])
    row2 = sy.Matrix([sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)])
    row3 = sy.Matrix([0, sy.sin(α), sy.cos(α), d])
    row4 = sy.Matrix([0, 0, 0, 1])
    return sy.Matrix([row1.T, row2.T, row3.T, row4.T])

## Setup Values ##
# Given robot link properties from DH table
a1 = 0.5
a2 = 0.5
d4 = 0.5

DHParams = sy.Matrix([[a1, 0, 0, sy.Symbol("Θ1")], [a2, sy.pi, 0, sy.Symbol("Θ2")], [0, 0, sy.Symbol("d3"), 0], [0, 0, d4, sy.Symbol("Θ4")]])

# Determine transformation matrices
T00 = sy.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T01 = DHSymbolic(DHParams[0, 0], DHParams[0, 1], DHParams[0, 2], DHParams[0, 3])
T12 = DHSymbolic(DHParams[1, 0], DHParams[1, 1], DHParams[1, 2], DHParams[1, 3])
T23 = DHSymbolic(DHParams[2, 0], DHParams[2, 1], DHParams[2, 2], DHParams[2, 3])
T34 = DHSymbolic(DHParams[3, 0], DHParams[3, 1], DHParams[3, 2], DHParams[3, 3])

T02 = T01 * T12
T03 = T02 * T23
T04 = T03 * T34

## Begin Jacobian ##
# Determine each joint jacobian
# Joint 1 (revolute)
J1 = sy.Matrix([T00[0:3, 2].cross(T04[0:3, 3] - T00[0:3, 3]), T00[0:3, 2]])

# Joint 2 (revolute)
J2 = sy.Matrix([T01[0:3, 2].cross(T04[0:3, 3] - T01[0:3, 3]), T01[0:3, 2]])

# Joint 3 (prismatic)
J3 = sy.Matrix([T02[0:3, 2], sy.zeros(3, 1)])

# Joint 4 (revolute)
J4 = sy.Matrix([T03[0:3, 2].cross(T04[0:3, 3] - T03[0:3, 3]), T03[0:3, 2]])

# Combine joint columns
Jsym = sy.Matrix([[J1.T], [J2.T], [J3.T], [J4.T]]).T

## Evaluate jacobian at joint position ##
# Define joint positions
theta1 = sy.pi / 4
theta2 = sy.pi / 4
d3 = 0.5
theta4 = sy.pi / 4

Jeval = Jsym.subs({sy.Symbol("Θ1"):theta1, sy.Symbol("Θ2"):theta2, sy.Symbol("d3"):d3, sy.Symbol("Θ4"):theta4})

Jround = sy.zeros(sy.shape(Jeval)[0], sy.shape(Jeval)[1])

for rowI in range(sy.shape(Jeval)[0]):
    for colI in range (sy.shape(Jeval)[1]):
        Jround[rowI, colI] = round(Jeval[rowI, colI], 4)

sy.pprint(Jround)