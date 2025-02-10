import numpy as np
import sympy as sy
import math

# Generate single rotation about X axis
def RotationXGenerator(theta):
    return np.array([[1, 0, 0], [0, math.cos(np.deg2rad(theta)), -math.sin(np.deg2rad(theta))], [0, math.sin(np.deg2rad(theta)), math.cos(np.deg2rad(theta))]])

# Generate single rotation about Y axis
def RotationYGenerator(theta):
    return np.array([[math.cos(np.deg2rad(theta)), 0, math.sin(np.deg2rad(theta))], [0, 1, 0], [-math.sin(np.deg2rad(theta)), 0, math.cos(np.deg2rad(theta))]])

# Generate single rotation about Z axis
def RotationZGenerator(theta):
    return np.array([[math.cos(np.deg2rad(theta)), -math.sin(np.deg2rad(theta)), 0], [math.sin(np.deg2rad(theta)), math.cos(np.deg2rad(theta)), 0], [0, 0, 1]])

def PreMultiplier(originalFrame, newFrame):
    return np.matmul(newFrame, originalFrame)

def PostMultiplier(originalFrame, newFrame):
    return np.matmul(originalFrame, newFrame)

def EulerAngles(RotationMatrix):
    # Find phi(rotation 1) using Euler
    phiEuler = np.rad2deg(math.atan2(RotationMatrix[1, 2], RotationMatrix[0, 2]))

    # Find theta(rotation 2) using Euler
    thetaEuler = np.rad2deg(math.atan2(math.sqrt(1 - RotationMatrix[2, 2]**2), RotationMatrix[2, 2]))

    # Find psi(rotation 3) using Euler
    psiEuler = np.rad2deg(math.atan2(RotationMatrix[2, 1], -RotationMatrix[2, 0]))

    return [phiEuler, thetaEuler, psiEuler]

def AxisAngle(RotationMatrix):
    theta = math.acos((np.trace(RotationMatrix) - 1) / 2)
    k = (1 / (2 * math.sin(np.deg2rad(theta)))) * np.array([[RotationMatrix[2, 1] - RotationMatrix[1, 2]], [RotationMatrix[0, 2] - RotationMatrix[2, 0]], [RotationMatrix[1, 0] - RotationMatrix[0, 1]]])
    return[k / np.linalg.norm(k), np.rad2deg(theta)]


# Question 4
print("\nQuestion 4")

# Generate the provided rotation matrix
Rot4 = np.array([[1 / math.sqrt(2), 0, 1/ math.sqrt(2)], [-0.5, 1 / math.sqrt(2), 0.5], [-0.5, -1 / math.sqrt(2), .5]])

# Find the inverse
Inverse4 = np.linalg.inv(Rot4)

# Find the transpose
Transpose4 = np.transpose(Rot4)

# Find the determinant
Determinant4 = np.linalg.det(Rot4)

# Find the norm of each row
NormRow1 = np.linalg.norm(Rot4[0, :])
NormRow2 = np.linalg.norm(Rot4[1, :])
NormRow3 = np.linalg.norm(Rot4[2, :])

# Find the norm of each column
NormCol1 = np.linalg.norm(Rot4[:, 0])
NormCol2 = np.linalg.norm(Rot4[:, 1])
NormCol3 = np.linalg.norm(Rot4[:, 2])

# Find the dot product of rows
DotRow12 = np.dot(Rot4[0, :], Rot4[1, :])
DotRow13 = np.dot(Rot4[0, :], Rot4[2, :])
DotRow23 = np.dot(Rot4[1, :], Rot4[2, :])

# Find the dot product of rows
DotCol12 = np.dot(Rot4[:, 0], Rot4[:, 1])
DotCol13 = np.dot(Rot4[:, 0], Rot4[:, 2])
DotCol23 = np.dot(Rot4[:, 1], Rot4[:, 2])

# Print all three matricies for comparison
print(f"The rotation matrix is:\n{Rot4}\n")

print(f"The inverse rotation matrix is:\n{Inverse4}\n")

print(f"The transpose rotation matrix is:\n{Transpose4}\n")

print(f"The determinant of the rotation matrix is:\n{np.round(Determinant4, 4)}\n")

print(f"The norm of the rows is:\n{np.array([NormRow1, NormRow2, NormRow3])}\n")

print(f"The norm of the columns is:\n{np.array([NormCol1, NormCol2, NormCol3])}\n")

print(f"The dot product of the rows are:\n{np.round(np.array([DotRow12, DotRow13, DotRow23]), 4)}\n")

print(f"The dot product of the columns are:\n{np.round(np.array([DotCol12, DotCol13, DotCol23]), 4)}\n")

# Determine the axis of rotation and angle for the rotation matrix
[kAA4, thetaAA4] = AxisAngle(Rot4)

# Print the unit vector and angle of rotation
print(f"\nThe unit vector axis of rotation is:\n{np.round(kAA4, 2)}")
print(f"\nThe angle of rotation in degrees is :\n{np.round(thetaAA4, 2)}")

# Determine the euler angles of the rotation matrix for ZYZ rotation
[phiEuler, thetaEuler, psiEuler] = EulerAngles(Rot4)

print(f'\nRotation 1(phi) about Z:\n{np.round(phiEuler, 2)} degrees\n')
print(f'Rotation 2(theta) about Y:\n{np.round(thetaEuler, 2)} degrees\n')
print(f'Rotation 3(psi) about Z:\n{np.round(psiEuler, 2)} degrees\n')


# Question 5
print("\nQuestion 5")

# Create rotation matricies for first 2 rotations
RotY = RotationYGenerator(90)
RotZ = RotationZGenerator(45)

# Post multiply current frame rotation to get final rotation matrix
Rot5 = PostMultiplier(RotY, RotZ)

[kAA5, thetaAA5] = AxisAngle(Rot5)

# Print the unit vector and angle of rotation
print(f"\nThe unit vector axis of rotation is:\n{np.round(kAA5, 2)}")
print(f"\nThe angle of rotation in degrees is :\n{np.round(thetaAA5, 2)}")

# Find the Euler ZYZ rotation matrix
RotZ1Euler = RotationZGenerator(90)
RotYEuler = RotationYGenerator(0)
RotZ2Euler = RotationZGenerator(45)

# Post multiply twice to obtain ZYZ rotation matrix
RotEuler5 = PostMultiplier(PostMultiplier(RotZ1Euler, RotYEuler), RotZ2Euler)

# Display the ZYZ rotation matrix
print(f"\nThe ZYZ rotation matrix is:\n{np.round(RotEuler5, 4)}\n")