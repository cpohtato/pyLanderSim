from .imports import *
from .consts import *

class TVCLanderState():
    def __init__(self, xMod, zMod, dxMod, dzMod, betaMod, dbetaMod, phiMod, dPhiMod):
        self.m = 1000
        self.x = -9260 * xMod
        self.z = 2600 * zMod
        self.dx = 179.42 * dxMod
        self.dz = -46.32 * dzMod
        self.beta = (-75.5 + betaMod) * math.pi / 180
        self.dbeta = dbetaMod * math.pi / 180
        self.phi = (0.0 + phiMod) * math.pi / 180
        self.dphi = dPhiMod * math.pi / 180

    def getVector(self):
        vector = [
            self.m,
            self.x,
            self.z,
            self.dx,
            self.dz,
            self.beta,
            self.dbeta,
            self.phi,
            self.dphi
        ]
        return vector
    
    def update(self, sol):
        self.m = sol[-1, 0]
        self.x = sol[-1, 1]
        self.z = sol[-1, 2]
        self.dx = sol[-1, 3]
        self.dz = sol[-1, 4]
        self.beta = sol[-1, 5]
        self.dbeta = sol[-1, 6]
        self.phi = sol[-1, 7]
        self.dphi = sol[-1, 8]

    def calculateAcceptableVertical(self, dx):
        return -(10.0 - 0.75 * abs(dx)) * 0.3048

    def softLanding(self, x):
        #   Refer to Apollo design limits here:
        #   https://www.ibiblio.org/apollo/Documents/TN-D-4131%20Lunar%20Module%20Pilot%20Control%20Considerations.pdf
        #   Numbers in document are in imperial; metric conversion required

        beta = x[5]
        dbeta = x[6]
        dx = x[3]
        dz = x[4]

        #   Attitude and rate constraints
        if (abs(beta) > 6.0 * math.pi / 180.0): return False
        # if (abs(dbeta) > 2.0 * math.pi / 180.0): return False

        #   Velocity constraints
        if (abs(dx) > 4.0 * 0.3048): return False
        acceptableVertical = self.calculateAcceptableVertical(dx)
        if (dz < acceptableVertical): return False

        return True
    
class TVCLander():
    def __init__(self, thrustMod, tauMod, xMod, zMod, dxMod, dzMod, betaMod, dbetaMod, phiMod, dPhiMod):
        self.m_max = 1000        #   kg
        self.m_min = 314.51      #   kg
        self.d_x = 0.75          #   m
        self.d_y = 0.9
        self.d_z = 0.75
        self.I_x = 551.25
        self.I_y = 551.25
        self.I_z = 281.25
        self.F_max = 4000
        self.I_spt = 289
        self.tau_max = 945.0
        self.b = 0.05
        self.J_x = 14.32
        self.J_y = 14.32
        self.phi_max = 6.0 * math.pi / 180.0
        self.state = TVCLanderState(xMod, zMod, dxMod, dzMod, betaMod, dbetaMod, phiMod, dPhiMod)

        self.prevDX = self.state.dx
        self.prevDZ = self.state.dz
        self.prevAccel = 0.0
        self.accumAccelError = 0.0

        self.currF = 0.0
        self.currTau = 0.0

        self.thrustMod = thrustMod
        self.F_max = self.F_max * thrustMod

        self.tauMod = tauMod
        self.tau_max = self.tau_max * tauMod

    def clampCommandedThrust(self):
        #   Clamp F
        if self.currF < 0:              self.currF = 0
        if self.currF > self.F_max:     self.currF = self.F_max
        if self.state.m <= 300:         self.currF = 0

    def calculateTargetVector(self, t):

        lowGateAltitude = 50.0
        distance = math.sqrt(pow(self.state.x, 2) + pow(self.state.z - lowGateAltitude, 2))

        if (self.state.z <= lowGateAltitude) or (distance <= 5.0):
            #   Low gate guidance, go straight down
            ACG_z = (-1.83 - self.state.dz)/5.0 + g
            ACG_x = -(self.state.dx/20.0)
        else:

            #   Following APDG
            #   https://pdf.sciencedirectassets.com/271426/1-s2.0-S0005109800X02579/1-s2.0-00
            #   https://doi.org/10.1016/0005-1098(74)90019-3

            T = 120 - t
            ACG_z = 12*(lowGateAltitude-self.state.z)/(pow(T, 2)) - 6*self.state.dz/T + g
            ACG_x = 12*(0.0-self.state.x)/(pow(T, 2)) - 6*self.state.dx/T     

        ACG_mag = math.sqrt(pow(ACG_z, 2) + pow(ACG_x, 2))
        ACG_angle = -math.atan2(ACG_z, ACG_x) + math.pi/2

        if (abs((ACG_angle - 2 * math.pi) - self.state.beta) < abs(ACG_angle - self.state.beta)):
            ACG_angle -= 2 * math.pi
        elif (abs((ACG_angle + 2 * math.pi) - self.state.beta) < abs(ACG_angle - self.state.beta)):
            ACG_angle += 2 * math.pi

        return ACG_mag, ACG_angle

    def digitalControlLoop(self, t):

        # if (r_z > 152):
            #   Before low gate

        ACG_mag, ACG_angle = self.calculateTargetVector(t)

        self.currF = 4000.0
        self.currTau = 0.0

        return ACG_mag, ACG_angle        

    def getState(self):
        state = self.state.getVector()
        return state

    def stateTransition3DoF(self, x, t):
        m, r_x, r_z, v_x, v_z, beta, dbeta, phi, dphi = x

        dxdt = [
            -self.currF/(self.I_spt * g_0),
            v_x,
            v_z,
            self.currF*np.sin(phi + beta)/m,
            self.currF*np.cos(phi + beta)/m - g,
            dbeta,
            -self.currTau/(self.I_y - pow(self.d_y, 2)*self.J_y)-(self.currF*np.sin(phi)*self.d_y)/self.I_y,
            dphi,
            (self.currTau - self.b*dphi)/self.J_y
        ]
        return dxdt
    
    def trajStateTransition(self, x, t):
        r_x, r_z, dx, dz, beta = x
        T = 120 - t

        dxdt = [
            dx,
            dz,
            12*(0.0-r_x)/(pow(T, 2)) - 6*dx/T,
            12*(50.0-r_z)/(pow(T, 2)) - 6*dz/T,
            0
        ]

        return dxdt

    def simulate(self):

        global g

        T_STEP = 0.1
        TOT_TIME = 180
        TIMESTEPS = round(TOT_TIME/0.1)
        n_horizon = 30

        trajectory = []
        ic = self.state.getVector()
        angle = math.atan2(ic[3], ic[4])
        angle -= math.pi
        trajIC = [ic[1], ic[2], ic[3], ic[4], angle]
        trajectory.append(trajIC)

        highGateSteps = math.floor(119.9/T_STEP)
        for i in range(1, highGateSteps):
            sol = odeint(self.trajStateTransition, trajectory[i-1], [(i-1)*0.1, i*0.1])
            angle = math.atan2(sol[-1, 2], sol[-1, 3])
            angle -= math.pi
            vector = [sol[-1, 0], sol[-1, 1], sol[-1, 2], sol[-1, 3], angle]
            trajectory.append(vector)

        traj_z = 50.0
        traj_dz = -1.83
        vector = [0.0, traj_z, 0.0, traj_dz, 0.0]      #   Singularity at T = 120
        trajectory.append(vector)

        while (traj_z > 1.0):
            traj_z += traj_dz * T_STEP
            vector = [0.0, traj_z, 0.0, traj_dz, 0.0]
            trajectory.append(vector)

        while (len(trajectory) < TIMESTEPS + n_horizon):
            vector = [0.0, traj_z, 0.0, 0.0, 0.0]
            trajectory.append(vector)

        arrR_x = []
        arrR_z = []
        arrR_dx = []
        arrR_dz = []
        arrR_beta = []

        for vec in trajectory:
            arrR_x.append(vec[0])
            arrR_z.append(vec[1])
            arrR_dx.append(vec[2])
            arrR_dz.append(vec[3])
            arrR_beta.append(vec[4])

        model = do_mpc.model.Model('continuous')

        r_x = model.set_variable(var_type='_tvp', var_name='r_x')
        r_z = model.set_variable(var_type='_tvp', var_name='r_z')
        r_dx = model.set_variable(var_type='_tvp', var_name='r_dx')
        r_dz = model.set_variable(var_type='_tvp', var_name='r_dz')
        r_beta = model.set_variable(var_type='_tvp', var_name='r_beta')

        m = model.set_variable(var_type='_x', var_name='m', shape=(1,1))
        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        z = model.set_variable(var_type='_x', var_name='z', shape=(1,1))
        dx = model.set_variable(var_type='_x', var_name='dx', shape=(1,1))
        dz = model.set_variable(var_type='_x', var_name='dz', shape=(1,1))
        beta = model.set_variable(var_type='_x', var_name='beta', shape=(1,1))
        dbeta = model.set_variable(var_type='_x', var_name='dbeta', shape=(1,1))
        phi = model.set_variable(var_type='_x', var_name='phi', shape=(1,1))
        dphi = model.set_variable(var_type='_x', var_name='dphi', shape=(1,1))

        thrustMod = model.set_variable('_p', 'thrustMod')
        
        F = model.set_variable(var_type='_u', var_name='F', shape=(1,1))
        tau = model.set_variable(var_type='_u', var_name='tau', shape=(1,1))

        model.set_rhs('m', -F * thrustMod/(self.I_spt * g_0))
        model.set_rhs('x', dx)
        model.set_rhs('z', dz)
        model.set_rhs('dx', F*np.sin(phi + beta)/m)
        model.set_rhs('dz', F*np.cos(phi + beta)/m - g)
        model.set_rhs('beta', dbeta)
        model.set_rhs('dbeta', -tau/(self.I_y-self.d_y**2*self.J_y)-F*np.sin(phi)*self.d_y/self.I_y)
        model.set_rhs('phi', dphi)
        model.set_rhs('dphi', (tau-self.b*dphi)/self.J_y)

        model.setup()

        mpc = do_mpc.controller.MPC(model)
        n_horizon = 50
        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': T_STEP,
            'n_robust': 0,
            'store_full_solution': True,
        }
        mpc.set_param(**setup_mpc)

        tvp_template = mpc.get_tvp_template()
        def tvp_fun(t):
            idx = math.floor(t/T_STEP)
            for k in range(n_horizon+1):
                tvp_template['_tvp', k, 'r_x'] = arrR_x[idx+k]
                tvp_template['_tvp', k, 'r_z'] = arrR_z[idx+k]
                tvp_template['_tvp', k, 'r_dx'] = arrR_dx[idx+k]
                tvp_template['_tvp', k, 'r_dx'] = arrR_dz[idx+k]
                tvp_template['_tvp', k, 'r_beta'] = arrR_beta[idx+k]
            return tvp_template
        

        mterm = 2*(r_x - x)**2 + 2*(r_z - z)**2 + (r_dx - dx)**2 + (r_dz - dz)**2
        lterm = mterm
        mpc.set_objective(mterm=mterm, lterm=lterm)

        mpc.set_rterm(
            F = 1e-5,
            tau = 1e-4
        )

        mpc.bounds['lower', '_u', 'F'] = 0.0
        mpc.bounds['upper', '_u', 'F'] = self.F_max
        mpc.bounds['lower', '_u', 'tau'] = -self.tau_max
        mpc.bounds['upper', '_u', 'tau'] = self.tau_max

        mpc.bounds['lower', '_x', 'm'] = self.m_min
        mpc.bounds['lower', '_x', 'z'] = 0.0
        mpc.bounds['lower', '_x', 'phi'] = -self.phi_max
        mpc.bounds['upper', '_x', 'phi'] = self.phi_max

        mpc.scaling['_x', 'm'] = 1000
        mpc.scaling['_x', 'x'] = 1000
        mpc.scaling['_x', 'z'] = 1000
        mpc.scaling['_x', 'dx'] = 100
        mpc.scaling['_x', 'dz'] = 100
        mpc.scaling['_x', 'beta'] = 10
        mpc.scaling['_x', 'dbeta'] = 10
        mpc.scaling['_x', 'phi'] = 0.1
        mpc.scaling['_x', 'dphi'] = 0.1
        mpc.scaling['_u', 'F'] = 1000
        mpc.scaling['_u', 'tau'] = 1000

        mpc.set_uncertainty_values(
            thrustMod = np.array([1., 0.95, 1.05])
        )

        mpc.set_tvp_fun(tvp_fun)
        mpc.setup()

        simulator = do_mpc.simulator.Simulator(model)
        simulator.set_param(t_step = 0.1)

        sim_tvp_template = simulator.get_tvp_template()
        def sim_tvp_fun(t):
            idx = math.floor(t/T_STEP)
            sim_tvp_template['r_x'] = arrR_x[idx]
            sim_tvp_template['r_z'] = arrR_z[idx]
            sim_tvp_template['r_dx'] = arrR_dx[idx]
            sim_tvp_template['r_dz'] = arrR_dz[idx]
            sim_tvp_template['r_beta'] = arrR_beta[idx]
            return sim_tvp_template
        simulator.set_tvp_fun(sim_tvp_fun)

        p_template = simulator.get_p_template()
        def p_fun(t):
            p_template['thrustMod'] = self.thrustMod
            return p_template
        simulator.set_p_fun(p_fun)

        simulator.setup()

        ic = self.state.getVector()
        x0 = np.array([ic[0], ic[1], ic[2], ic[3], ic[4], ic[5], ic[6], ic[7], ic[8]]).reshape(-1,1)
        simulator.x0 = x0
        mpc.x0 = x0
        mpc.set_initial_guess()

        mpl.rcParams['font.size'] = 8
        mpl.rcParams['lines.linewidth'] = 2
        mpl.rcParams['axes.grid'] = True
        
        mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
        sim_graphics = do_mpc.graphics.Graphics(simulator.data)

        fig, ax = plt.subplots(3, 3, sharex=True, figsize=(10,8))
        fig.align_ylabels()

        for g in [sim_graphics]:
            g.add_line(var_type='_x', var_name='x', axis=ax[0,0])
            g.add_line(var_type='_x', var_name='z', axis=ax[0,0])

            g.add_line(var_type='_x', var_name='dx', axis=ax[1,0])
            g.add_line(var_type='_x', var_name='dz', axis=ax[1,0])

            g.add_line(var_type='_u', var_name='F', axis=ax[2,0])

            g.add_line(var_type='_x', var_name='beta', axis=ax[0,1])

            g.add_line(var_type='_x', var_name='dbeta', axis=ax[1,1])

            g.add_line(var_type='_x', var_name='m', axis=ax[2,1])

            g.add_line(var_type='_x', var_name='phi', axis=ax[0,2])

            g.add_line(var_type='_x', var_name='dphi', axis=ax[1,2])

            g.add_line(var_type='_u', var_name='tau', axis=ax[2,2])

        ax[0,0].set_ylabel('Distance [m]')
        lines = sim_graphics.result_lines['_x', 'x'] + sim_graphics.result_lines['_x', 'z']
        ax[0,0].legend(lines, ['x', 'z'])

        ax[1,0].set_ylabel('Velocity [m/s]')
        lines = sim_graphics.result_lines['_x', 'dx'] + sim_graphics.result_lines['_x', 'dz']
        ax[1,0].legend(lines, ['dx', 'dz'])

        ax[2,0].set_ylabel('Thrust [N]')
        ax[2,0].legend(sim_graphics._result_lines['_u', 'F'], ['F'])
        ax[2,0].set_xlabel('Time [s]')

        ax[0,1].set_ylabel('Angle [rad]')
        ax[0,1].legend(sim_graphics._result_lines['_x', 'beta'], ['beta'])

        ax[1,1].set_ylabel('Velocity [rad/s]')
        ax[1,1].legend(sim_graphics._result_lines['_x', 'dbeta'], ['dbeta'])

        ax[2,1].set_ylabel('Mass [kg]')
        ax[2,1].legend(sim_graphics._result_lines['_x', 'm'], ['m'])
        ax[2,1].set_xlabel('Time [s]')

        ax[0,2].set_ylabel('Gimbal Angle [rad]')
        ax[0,2].legend(sim_graphics._result_lines['_x', 'phi'], ['phi'])

        ax[1,2].set_ylabel('Gimbal Velocity [rad/s]')
        ax[1,2].legend(sim_graphics._result_lines['_x', 'dphi'], ['dphi'])

        ax[2,2].set_ylabel('Torque [Nm]')
        ax[2,2].legend(sim_graphics._result_lines['_u', 'tau'], ['tau'])
        ax[2,2].set_xlabel('Time [s]')

        simulator.reset_history()
        simulator.x0 = x0
        x = x0
        mpc.reset_history()

        for i in range(TIMESTEPS):
            u = mpc.make_step(x)
            x = simulator.make_step(u)

            if (x[2] <= 1.0): break

            print("========================================================================")
            do_mpc.tools.printProgressBar(i, TIMESTEPS)
            print()
            print("z-pos: " + str(round(x[2,0], 2)) + " [m]")
            print("========================================================================")

        print("Soft landing: " + str(self.state.softLanding(x)))
        print("Final x: " + str(x[1,0]) + " [m]")
        print("Fuel consumed: " + str(1000.0 - x[0,0]) + " [kg]")

        # mpc_graphics.plot_predictions(t_ind=0)
        sim_graphics.plot_results()
        sim_graphics.reset_axes()

        plt.show()

        # numSteps = round(SIM_LENGTH/DT)+1
        # t = np.linspace(0, SIM_LENGTH, numSteps)

        # successfulLanding = False
        # solution = []
        # initCondition = self.state.getVector()
        # initCondition.append(0.0)   #   t
        # ACG_mag, ACG_angle = self.calculateTargetVector(0.0)
        # initCondition.append(ACG_mag)   #   ACG_mag
        # initCondition.append(ACG_angle)   #   ACG_angle
        # solution.append(initCondition)

        # for idx in range(len(t)-1):
        #     r_thrust, r_angle = self.digitalControlLoop(t[idx])
        #     step = t[idx:idx+2]
        #     sol = odeint(self.stateTransition3DoF, self.getState(), step)
        #     self.state.update(sol)
        #     vector = self.state.getVector()
        #     vector.append(t[idx+1])
        #     vector.append(r_thrust)
        #     vector.append(r_angle)
        #     solution.append(vector)

        #     if (self.state.z <= 1.0):
        #         successfulLanding = self.state.softLanding()
        #         break

        # fuelConsumed = self.m_max - self.state.m

        # return successfulLanding, self.state.x, fuelConsumed, solution