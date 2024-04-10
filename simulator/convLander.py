from .imports import *
from .consts import *

class ConvLanderState():
    def __init__(self, xMod, zMod, dxMod, dzMod, betaMod, dbetaMod):
        self.m = 1000
        self.x = -9260 * xMod
        self.z = 2600 * zMod
        self.dx = 179.42 * dxMod
        self.dz = -46.32 * dzMod
        self.beta = (-75.5 + betaMod) * math.pi / 180
        self.dbeta = dbetaMod * math.pi / 180

    def getVector(self):
        vector = [
            self.m,
            self.x,
            self.z,
            self.dx,
            self.dz,
            self.beta,
            self.dbeta
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

    def calculateAcceptableVertical(self):
        return -(10.0 - 0.75 * abs(self.dx)) * 0.3048

    def softLanding(self):
        #   Refer to Apollo design limits here:
        #   https://www.ibiblio.org/apollo/Documents/TN-D-4131%20Lunar%20Module%20Pilot%20Control%20Considerations.pdf
        #   Numbers in document are in imperial; metric conversion required

        #   Attitude and rate constraints
        if (abs(self.beta) > 6.0 * math.pi / 180.0): return False
        if (abs(self.dbeta) > 2.0 * math.pi / 180.0): return False

        #   Velocity constraints
        if (abs(self.dx) > 4.0 * 0.3048): return False
        acceptableVertical = self.calculateAcceptableVertical()
        if (self.dz < acceptableVertical): return False

        return True

class ConvLander():
    def __init__(self, thrustMod, reactionMod, xMod, zMod, dxMod, dzMod, betaMod, dbetaMod):
        self.m_max = 1000        #   kg
        self.m_min = 300         #   kg
        self.d_x = 0.75          #   m
        self.d_y = 0.9
        self.d_z = 0.75
        self.I_x = 551.25
        self.I_y = 551.25
        self.I_z = 281.25
        self.F_max = 4000
        self.I_spt = 289
        self.M_x_max = 150
        self.M_y_max = 180
        self.M_z_max = 150
        self.I_spm = 280
        self.state = ConvLanderState(xMod, zMod, dxMod, dzMod, betaMod, dbetaMod)

        self.prevDX = self.state.dx
        self.prevDZ = self.state.dz
        self.prevAccel = 0.0
        self.accumAccelError = 0.0

        self.currF = 0.0
        self.currM_x = 0.0
        self.currM_y = 0.0
        self.currM_z = 0.0

        self.thrustMod = thrustMod
        self.F_max = self.F_max * thrustMod

        self.reactionMod = reactionMod
        self.M_y_max = self.M_y_max * reactionMod

    def clampCommandedThrust(self):
        #   Clamp F
        if self.currF < 0:              self.currF = 0
        if self.currF > self.F_max:     self.currF = self.F_max
        if self.state.m <= 300:         self.currF = 0

    def clampCommandedMoment(self):
        #   Clamp M_y
        if self.currM_y > self.M_y_max:     self.currM_y = self.M_y_max
        if self.currM_y < -self.M_y_max:    self.currM_y = -self.M_y_max
        if self.state.m <= 300:             self.currM_y = 0

    def calculateTargetVector(self, t):

        T = 120 - t
        ACG_z = -12*(2+self.state.z)/(pow(T, 2)) - 6*self.state.dz/T + g

        if (self.state.z > 152):
            #   Following APDG
            #   https://pdf.sciencedirectassets.com/271426/1-s2.0-S0005109800X02579/1-s2.0-00
            #   https://doi.org/10.1016/0005-1098(74)90019-3
            
            ACG_x = 12*(324.14-self.state.x)/(pow(T, 2)) - 6*self.state.dx/T

        else:
            #   Kill off any horizontal movement now
            ACG_x = -(self.state.dx/5.0)

        ACG_mag = math.sqrt(pow(ACG_z, 2) + pow(ACG_x, 2))
        ACG_angle = -math.atan2(ACG_z, ACG_x) + math.pi/2

        return ACG_mag, ACG_angle

    def digitalControlLoop(self, t):

        # if (r_z > 152):
            #   Before low gate

        ACG_mag, ACG_angle = self.calculateTargetVector(t)

        #   Main engine feedback controller
        ddx = (self.state.dx - self.prevDX)/DT
        ddz = (self.state.dz - self.prevDZ)/DT
        dr_mag = math.sqrt(pow(ddx, 2) + pow(ddz, 2))
        accelError = self.prevAccel - dr_mag
        self.accumAccelError += accelError * DT

        self.currF = self.state.m * ACG_mag * self.thrustMod
        Kp_t = 1.0
        Ki_t = 1.0
        self.currF += Kp_t * accelError * self.thrustMod
        self.currF += Ki_t * self.accumAccelError * self.thrustMod

        #   Angular feedback controller
        Kp_r = self.I_y
        Kd_r = 2 * self.I_y
        err = ACG_angle - self.state.beta
        self.currM_y = Kp_r * err - Kd_r * self.state.dbeta * self.reactionMod
        
        self.clampCommandedThrust()
        self.clampCommandedMoment()

        self.prevDX = self.state.dx
        self.prevDZ = self.state.dz
        self.prevAccel = ACG_mag

        return ACG_mag, ACG_angle        

    def getState(self):
        state = self.state.getVector()
        return state

    def stateTransition3DoF(self, x, t):
        m, r_x, r_z, v_x, v_z, beta, dbeta = x

        dxdt = [
            -self.currF/(self.I_spt * g_0) - self.currM_y/(self.d_y * self.I_spm * g_0),
            v_x,
            v_z,
            self.currF*np.sin(beta)/m,
            self.currF*np.cos(beta)/m - g,
            dbeta,
            self.currM_y/self.I_y 
        ]
        return dxdt
    
    def simulate(self):
        numSteps = round(SIM_LENGTH/DT)+1
        t = np.linspace(0, SIM_LENGTH, numSteps)

        successfulLanding = False
        solution = []
        initCondition = self.state.getVector()
        initCondition.append(0.0)   #   t
        ACG_mag, ACG_angle = self.calculateTargetVector(0.0)
        initCondition.append(ACG_mag)   #   ACG_mag
        initCondition.append(ACG_angle)   #   ACG_angle
        solution.append(initCondition)

        for idx in range(len(t)-1):
            r_thrust, r_angle = self.digitalControlLoop(t[idx])
            step = t[idx:idx+2]
            sol = odeint(self.stateTransition3DoF, self.getState(), step)
            self.state.update(sol)
            vector = self.state.getVector()
            vector.append(t[idx+1])
            vector.append(r_thrust)
            vector.append(r_angle)
            solution.append(vector)

            if (self.state.z <= 1.0):
                successfulLanding = self.state.softLanding()
                fuelConsumed = self.m_max - self.state.m
                # acceptableVertical = self.state.calculateAcceptableVertical()

                # print()
                # if (successfulLanding):
                #     print("Successfully soft landed!")
                # else:
                #     print("Crash landed")

                # print()
                # print("Terminal conditions:")
                # print("Vertical velocity: " + str(round(self.state.dz, 2)) + "/" + str(round(acceptableVertical, 2)) + " [m/s]")
                # print("Horizontal velocity: " + str(round(self.state.dx, 2)) + "/1.22 [m/s]")
                # print("Angle from normal: " + str(round(self.state.beta * 180.0 / math.pi, 2)) + "/6.0 [deg]")
                # print("Angular velocity: " + str(round(self.state.dbeta * 180.0 / math.pi, 2)) + "/2.0 [deg/s]")

                # print("Horizontal error: " + str(round(self.state.x, 2)) + " [m]")
                # print()

                break

        return successfulLanding, self.state.x, fuelConsumed, solution