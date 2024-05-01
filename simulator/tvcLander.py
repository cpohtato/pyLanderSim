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
    
class TVCLander():
    def __init__(self, thrustMod, tauMod, xMod, zMod, dxMod, dzMod, betaMod, dbetaMod, phiMod, dPhiMod):
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
        self.tau_max = 0.0
        self.b = 0.0
        self.J_x = 0.0
        self.J_y = 0.0
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

        self.currF = 0.0
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
            -self.currTau/(self.I_y - pow(self.dy, 2)*self.J_y)-(self.currF*np.sin(phi))/self.I_y,
            dphi,
            (self.currTau - self.b*dphi)/self.J_y
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

                break

        fuelConsumed = self.m_max - self.state.m

        return successfulLanding, self.state.x, fuelConsumed, solution