import numpy as np
import matplotlib.pyplot as plt
import math

# Setup variables
time0 = 0 # initial time
accelBlend = 50 # parabolic blend acceleration

initPos1 = 10 # initial position in degrees
finalPos1 = 35 # final position in degrees
initVel1 = 0 # initial velocity in degrees/second
finalVel1 = 0 # final velocity in degrees/second
time1 = 2 # movement time in seconds from start

initPos2 = finalPos1 # initial position in degrees
finalPos2 = 25 # final position in degrees
initVel2 = 0 # initial velocity in degrees/second
finalVel2 = 0 # final velocity in degrees/second
time2 = 3 # movement time in seconds from start

posA = np.array([])
velA = np.array([])
accelA = np.array([])

# Compute cubic for move 1 part a
A1A = np.array([[1, time0, time0**2, time0**3], [0, 1, 2*time0, 3*time0**2], [1, time1, time1**2, time1**3], [0, 1, 2*time1, 3*time1**2]])

b1A = np.array([[initPos1], [initVel1], [finalPos1], [finalVel1]])

alpha1A = np.matmul(np.linalg.inv(A1A), b1A)

# Compute cubic for move 2 part a
A2A = np.array([[1, time1, time1**2, time1**3], [0, 1, 2*time1, 3*time1**2], [1, time2, time2**2, time2**3], [0, 1, 2*time2, 3*time2**2]])

b2A = np.array([[initPos2], [initVel2], [finalPos2], [finalVel2]])

alpha2A = np.matmul(np.linalg.inv(A2A), b2A)

# Compute position, velocity, acceleration vs time
time = np.linspace(time0, time2, 1000)

for i in time:
    if i < 2:
        # move 1
        posA = np.append(posA, alpha1A[3] * i**3 + alpha1A[2] * i**2 + alpha1A[1] * i + alpha1A[0])
        velA = np.append(velA, 3 * alpha1A[3] * i**2 + 2 * alpha1A[2] * i + alpha1A[1])
        accelA = np.append(accelA, 6 * alpha1A[3] * i + 2 * alpha1A[2])
    else:
        # move 2
        posA = np.append(posA, alpha2A[3] * i**3 + alpha2A[2] * i**2 + alpha2A[1] * i + alpha2A[0])
        velA = np.append(velA, 3 * alpha2A[3] * i**2 + 2 * alpha2A[2] * i + alpha2A[1])
        accelA = np.append(accelA, 6 * alpha2A[3] * i + 2 * alpha2A[2])

# Plot position, velocity, acceleration part a
fig1 = plt.figure(1)
plt.plot(time, posA)
plt.xlabel("Time (s)")
plt.ylabel("Position (degrees)")
plt.title("Cubic Position vs Time")

fig2 = plt.figure(2)
plt.plot(time, velA)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (degrees/sec)")
plt.title("Cubic Velocity vs Time")

fig3 = plt.figure(3)
plt.plot(time, accelA)
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (degrees/sec^2)")
plt.title("Cubic Acceleration vs Time")

# Part b
posB = np.array([])
velB = np.array([])
accelB = np.array([])

# Compute tB for move 1 and 2
tB1P = (accelBlend * time1 + math.sqrt((accelBlend * time1)**2 - 4 * accelBlend * (finalPos1 - initPos1))) / (2 * accelBlend) # time before constant velocity in parabolic blend
tB2P = (-accelBlend * (time2 - time1) + math.sqrt((-accelBlend * (time2 - time1))**2 - 4 * -accelBlend * (finalPos2 - initPos2))) / (2 * -accelBlend) # time before constant velocity in parabolic blend

tB1N = (accelBlend * time1 - math.sqrt((accelBlend * time1)**2 - 4 * accelBlend * (finalPos1 - initPos1))) / (2 * accelBlend) # time before constant velocity in parabolic blend
tB2N = (-accelBlend * (time2 - time1) - math.sqrt((-accelBlend * (time2 - time1))**2 - 4 * -accelBlend * (finalPos2 - initPos2))) / (2 * -accelBlend) # time before constant velocity in parabolic blend

if tB1N < tB1P and tB1N > 0:
    tB1 = tB1N
elif tB1P > 0:
    tB1 = tB1P
else:
    tB1 = tB1N

if tB2N < tB2P and tB2N > 0:
    tB2 = tB2N
elif tB2P > 0:
    tB2 = tB2P
else:
    tB2 = tB2N

# Compute linear blend move 1 part b
for i in time:
    if i <= tB1:
        # move 1 accel
        posB = np.append(posB, initPos1 + (accelBlend / 2) * i**2)
        velB = np.append(velB, accelBlend * i)
        accelB = np.append(accelB, accelBlend)
    elif i <= time1 - tB1:
        # move 1 const
        posB = np.append(posB, ((finalPos1 + initPos1 - (accelBlend * tB1 * time1)) / 2) + (accelBlend * tB1 * i))
        velB = np.append(velB, accelBlend * tB1)
        accelB = np.append(accelB, 0)
    elif i <= time1:
        # move 1 decel
        posB = np.append(posB, finalPos1 - ((accelBlend / 2) * time1**2) + accelBlend * i * time1 - (accelBlend / 2) * i**2)
        velB = np.append(velB, accelBlend * time1 - accelBlend * i)
        accelB = np.append(accelB, -accelBlend)

    elif i <= time1 + tB2:
        # move 2 accel
        posB = np.append(posB, initPos2 - (accelBlend / 2) * (i - time1)**2)
        velB = np.append(velB, -accelBlend * (i - time1))
        accelB = np.append(accelB, -accelBlend)
    elif i <= time2 - tB2:
        # move 2 const
        posB = np.append(posB, ((finalPos2 + initPos2 + (accelBlend * tB2 * (time2 - time1))) / 2) - (accelBlend * tB2 * (i - time1)))
        velB = np.append(velB, -accelBlend * tB2)
        accelB = np.append(accelB, 0)
    else:
        # move 2 decel
        posB = np.append(posB, finalPos2 + ((accelBlend / 2) * (time2 - time1)**2) - accelBlend * (i - time1) * (time2 - time1) + (accelBlend / 2) * (i - time1)**2)
        velB = np.append(velB, -accelBlend * (time2 - time1) + accelBlend * (i - time1))
        accelB = np.append(accelB, accelBlend)

# Plot position, velocity, acceleration part b
fig1 = plt.figure(4)
plt.plot(time, posB)
plt.xlabel("Time (s)")
plt.ylabel("Position (degrees)")
plt.title("Blend Position vs Time")

fig2 = plt.figure(5)
plt.plot(time, velB)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (degrees/sec)")
plt.title("Blend Velocity vs Time")

fig3 = plt.figure(6)
plt.plot(time, accelB)
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (degrees/sec^2)")
plt.title("Blend Acceleration vs Time")
plt.show()