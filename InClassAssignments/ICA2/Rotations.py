import numpy as np
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

def Euler(RotationMatrix):
    # Find phi(rotation 1) using Euler
    phiEuler = np.rad2deg(math.atan2(RotationMatrix[1, 2], RotationMatrix[0, 2]))

    # Find theta(rotation 2) using Euler
    thetaEuler = np.rad2deg(math.atan2(math.sqrt(1 - RotationMatrix[2, 2]**2), RotationMatrix[2, 2]))

    # Find psi(rotation 3) using Euler
    psiEuler = np.rad2deg(math.atan2(RotationMatrix[2, 1], -RotationMatrix[2, 0]))

    return [phiEuler, thetaEuler, psiEuler]

## Q1
# Define angles of rotation about y and z axis
thetaY = 45
thetaZ = 45

# Define rotation matrix for single axis rotation
RotY = RotationYGenerator(thetaY)
RotZ = RotationZGenerator(thetaZ)

# Multiply rotation matrix using post-multiplication to find current frame rotation
RotationCurrent = PostMultiplier(RotY, RotZ)

# Multiply rotation matrix using pre-multiplication to find fixed frame rotation
RotationFixed = PreMultiplier(RotY, RotZ)

# Display the current and fixed frame rotation matricies
print('Question 1:\n')
print(f'Current frame rotation:\n{np.round(RotationCurrent, 4)}\n')
print(f'Fixed frame rotation:\n{np.round(RotationFixed, 4)}\n')


## Q2
# Define Euler ZYZ rotation matrix
Rotation = np.array([[0.5, -0.5, 0.7071], [0.7071, 0.7071, 0], [-0.5, 0.5, 0.7071]])

# Find the Euler angles of rotation
eulerAngles = Euler(Rotation)

# Display rotation angles
print('\n\nQuestion 2:\n')
print(f'Rotation 1(phi) about Z:\n{np.round(eulerAngles[0], 2)} degrees\n')
print(f'Rotation 2(theta) about Y:\n{np.round(eulerAngles[1], 2)} degrees\n')
print(f'Rotation 3(psi) about Z:\n{np.round(eulerAngles[2], 2)} degrees\n')

# Find the rotation matrix to verify Euler method is correct
RotationCheck = PostMultiplier(PostMultiplier(RotationZGenerator(eulerAngles[0]), RotationYGenerator(eulerAngles[1])), RotationZGenerator(eulerAngles[2]))

# Display the verification matrix
print(f'Verification matrix:\n{np.round(RotationCheck, 4)}')