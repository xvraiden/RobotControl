import sympy as sy
import matplotlib.pyplot as plt

def ForwardKinematics(j1, j2):
    xLink1 = l1 * sy.cos(j1)
    yLink1 = l1 * sy.sin(j1)

    xlink2 = xLink1 + l2 * sy.cos(j1 + j2)
    ylink2 = yLink1 + l2 * sy.sin(j1 + j2)
    return sy.Matrix([[xLink1, yLink1], [xlink2, ylink2]])

def DetermineAccels(j1, j2, j1V, j2V):
    eq1 = (m1 * lc1**2 + m2 * l1**2 + 2 * m2 * l1 * lc2 * sy.cos(j2) + m2 * lc2**2 + I1 + I2) * q1DDot + (m2 * l1 * lc2 * sy.cos(j2) + m2 * lc2**2 + I2) * q2DDot - 2 * m1 * l1 * lc2 * sy.sin(j2) * j1V * j2V - m2 * l1 * lc2 * sy.sin(j2) * j2V**2 + m1 * 9.81 * lc1 * sy.cos(j1) + m2 * 9.81 * l1 * sy.cos(j1) + m2 * 9.81 * lc2 * sy.cos(j1 + j2) - tau1
    eq2 = (m2 * l1 * lc2 * sy.cos(j2) + m2 * lc2**2 + I2) * q1DDot + (m2 * lc2**2 + I2) * q2DDot + m2 * l1 * lc2 * sy.sin(j2) * j1V**2 + m2 * 9.81 * lc2 * sy.cos(j1 + j2) - tau2

    solutions = sy.solve([eq1, eq2], (q1DDot, q2DDot))
    
    j1DDot = solutions[q1DDot]
    j2DDot = solutions[q2DDot]

    return sy.Matrix([j1DDot, j2DDot])

def DetermineVels(j1V, j2V, j1A, j2A):
    j1Dot = j1V + j1A * dt
    j2Dot = j2V + j2A * dt
    return sy.Matrix([j1Dot, j2Dot])

def DeterminePoss(j1, j2, j1V, j2V, j1A, j2A):
    j1 = j1 + j1V * dt + 0.5 * j1A * dt**2
    j2 = j2 + j2V * dt + 0.5 * j2A * dt**2
    return sy.Matrix([j1, j2])

# Mechanical properties
m1 = 0.5 # mass of link 1
m2 = 0.5 # mass of link 2
l1 = 0.2 # length of link 1
l2 = 0.2 # length of link 2
lc1 = 0.1 # position of CM of link 1
lc2 = 0.1 # position of CM of link 2
I1 = m1 * l1**2 / 12; # moment of inertia of link 1
I2 = m2 * l2**2 / 12; # moment of inertia of link 2
dt = 0.01 # time step size

#Initial conditions - vertically down position, and zero velocity and acceleration
q = sy.Matrix([-sy.pi/2, 0])
q_dot = sy.Matrix([0, 0])
q_ddot = sy.Matrix([0, 0])

# Constant joint torques applied
tau1 = 0.2
tau2 = 0.2

# Solver properties
t = 0 # solve time
tFinal = 10 # end time

# Symbols
q1DDot = sy.Symbol("q1DDot")
q2DDot = sy.Symbol("q2DDot")

linkPos = ForwardKinematics(q[0], q[1])
robotPlot = plt.plot([0, linkPos[0, 0]], [0, linkPos[0, 1]], "-og", [linkPos[0, 0], linkPos[1,0]], [linkPos[0, 1], linkPos[1, 1]], "-ob")
plt.title("Robot Workspace")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.xlim([-(l1 + l2) * 1.5, (l1 + l2) * 1.5])
plt.ylim([-(l1 + l2) * 1.5, (l1 + l2) * 1.5])
timeText = plt.text(-0.4, 0.4, f"Time = {0} seconds")

plt.pause(0.01)

while t <= tFinal:
    q_ddot = DetermineAccels(q[0], q[1], q_dot[0], q_dot[1])
    q = DeterminePoss(q[0], q[1], q_dot[0], q_dot[1], q_ddot[0], q_ddot[1])
    q_dot = DetermineVels(q_dot[0], q_dot[1], q_ddot[0], q_ddot[1])
    linkPos = ForwardKinematics(q[0], q[1])
    robotPlot[0].set_data([0, linkPos[0, 0]], [0, linkPos[0, 1]])
    robotPlot[1].set_data([linkPos[0, 0], linkPos[1,0]], [linkPos[0, 1], linkPos[1, 1]])
    timeText.set_text(f"Time = {t} seconds")
    plt.pause(0.0001)
    t = t + dt

plt.show()