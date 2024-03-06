import math
import matplotlib
matplotlib.use('tkagg')
from matplotlib import pyplot as plt
import numpy as np
from scipy.integrate import odeint

#   Simulation parameters
SIM_LENGTH = 150     #   s
DT = 0.01           #   s

#   Conventional lander specs
m_max = 1000        #   kg
m_min = 300         #   kg
d_x = 0.75          #   m
d_y = 0.9
d_z = 0.75
I_x = 551.25
I_y = 551.25
I_z = 281.25
F_max = 4000
I_spt = 289
M_x_max = 150
M_y_max = 180
M_z_max = 150
I_spm = 280
g_0 = 9.81
g = 1.62

#   Constant control inputs, change later
M_x = 0
M_y = 0
M_z = 0

#   Initial conditions

x_init = [
    1000,
    0,
    2600,
    0,
    -46.32,
    0,
    0
]

numSteps = round(SIM_LENGTH/DT)+1
t = np.linspace(0, SIM_LENGTH, numSteps)

def stateTransition3DoF(x, t):
    m, r_x, r_z, v_x, v_z, beta, dbeta = x

    if (r_z > 152):
        #   Before low gate
        #   P velocity controller with set point at v_z = -20
        Kp = -m
        F = Kp * (v_z + 20) + m * g
    else:
        #   After low gate
        #   PD altitude controller with set point at r_z = 0
        #   Damping ratio is 2
        Kp = -m * 0.25
        Kd = -2 * 1 * m
        F = Kp * r_z + Kd * v_z + m * g

    #   Clamp F
    if F < 0:       F = 0
    if F > F_max:   F = F_max
    if m <= 300:    F = 0

    # F = 1000

    dxdt = [
        -F/(I_spt * g_0) - M_y/(d_y * I_spm * g_0),
        v_x,
        v_z,
        F*np.sin(beta)/m,
        F*np.cos(beta)/m - g,
        dbeta,
        M_y/I_y 
    ]
    return dxdt

sol = odeint(stateTransition3DoF, x_init, t)

# fig, axs = plt.subplots(3)
# fig.suptitle('State variables')

# axs[0].plot(t, sol[:, 0], label='Mass [kg]')
# axs[0].legend(loc='best')
# axs[0].set_title("Mass")

# axs[1].plot(t, sol[:, 4], label='dz [m/s]')
# axs[1].plot(t, sol[:, 3], label='dx [m/s]')
# axs[1].legend(loc='best')
# axs[1].set_title("Velocities")

# axs[2].plot(t, sol[:, 2], label='z [m]')
# axs[2].plot(t, sol[:, 1], label='x [m]')
# axs[2].legend(loc='best')
# axs[2].set_title("Position")
# axs[2].set(xlabel='t')

# axs[3].plot(t, sol[:, 5], label='γ [m]')
# axs[3].legend(loc='best')
# axs[3].set_title("Pitch")

# axs[4].plot(t, sol[:, 6], label='dγ [m/s]')
# axs[4].legend(loc='best')
# axs[4].set_title("Pitch Rate")
# axs[4].set(xlabel='t')

# for ax in axs.flat:
#     ax.label_outer()

# plt.figure(2)
# plt.plot(sol[:, 1], sol[:, 2])
# plt.title("Flight Path")
# plt.xlabel("m")
# plt.ylabel("y")

# plt.figure(1)
# plt.plot(t, sol[:, 0], label='Mass [kg]')
# plt.legend(loc='best')
# plt.title("Mass")
# plt.xlabel('t [s]')

# plt.figure(2)
# plt.plot(t, sol[:, 4], label='dz [m/s]')
# plt.plot(t, sol[:, 3], label='dx [m/s]')
# plt.legend(loc='best')
# plt.title("Velocity Components")
# plt.xlabel('t [s]')

plt.figure()
plt.plot(t, sol[:, 2], label='z')
plt.plot(t, sol[:, 1], label='x')
plt.legend(loc='best')
plt.xlabel('t [s]')
plt.ylabel('[m]')

# plt.figure(2)
# plt.plot(t, sol[:, 1], label='x [m]')
# plt.plot(t, sol[:, 3], label='dx [m/s]')
# plt.legend(loc='best')
# plt.xlabel('t')

# plt.figure(3)
# plt.plot(t, sol[:, 2], label='z [m]')
# plt.plot(t, sol[:, 4], label='dz [m/s]')
# plt.legend(loc='best')
# plt.xlabel('t')

# plt.figure(4)
# plt.plot(t, sol[:, 5], label='γ [rad]')
# plt.plot(t, sol[:, 6], label='dγ [rad/s]')
# plt.legend(loc='best')
# plt.title("Pitch")
# plt.xlabel('t [s]')

# plt.figure(5)
# totalTimesteps = round(SIM_LENGTH/DT)+1
# numTimestepsPerInterval = math.floor(totalTimesteps/256)
# for step in range(0, 256):
#     plt.scatter(sol[step * numTimestepsPerInterval, 1], 
#                 sol[step * numTimestepsPerInterval, 2], 
#                 c=[[(255-step)/255.0, step/255.0,  0]])
# plt.title("Flight Path")
# plt.xlabel("x [m]")
# plt.ylabel("z [m]")

plt.show()