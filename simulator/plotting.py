from .imports import *

def plotResults(sol):

    m = []
    x = []
    z = []
    dx = []
    dz = []
    beta = []
    dbeta = []
    t = []

    for timestep in sol:
        m.append(timestep[0])
        x.append(timestep[1])
        z.append(timestep[2])
        dx.append(timestep[3])
        dz.append(timestep[4])
        beta.append(timestep[5])
        dbeta.append(timestep[6])
        t.append(timestep[7])

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
    plt.plot(t, z, label='z')
    plt.plot(t, x, label='x')
    plt.legend(loc='best')
    plt.xlabel('t [s]')
    plt.ylabel('[m]')

    # plt.figure(2)
    # plt.plot(t, sol[:, 1], label='x [m]')
    # plt.plot(t, sol[:, 3], label='dx [m/s]')
    # plt.legend(loc='best')
    # plt.xlabel('t')

    plt.figure()
    plt.plot(t, z, label='z [m]')
    plt.plot(t, dz, label='dz [m/s]')
    plt.legend(loc='best')
    plt.xlabel('t')

    plt.figure()
    plt.plot(t, beta, label='β [rad]')
    plt.plot(t, dbeta, label='dβ [rad/s]')
    plt.legend(loc='best')
    plt.title("Pitch")
    plt.xlabel('t [s]')

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