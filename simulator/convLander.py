from .imports import *
from .consts import *

class ConvLanderState():
    def __init__(self):
        self.m = 1000
        self.x = -9260
        self.z = 2600
        self.dx = 179.42
        self.dz = -46.32
        self.beta = -75.5 * math.pi / 180
        self.dbeta = 0

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

class ConvLander():
    def __init__(self):
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
        self.state = ConvLanderState()

    def getIC(self):
        ic = self.state.getVector()
        return ic

    def stateTransition3DoF(self, x, t):
        m, r_x, r_z, v_x, v_z, beta, dbeta = x

        M_x = 0
        M_y = 0
        M_z = 0

        # if (r_z > 152):
            #   Before low gate
        #   Following APDG

        T = 100 - t
        ACG_z = -12*r_z/(pow(T, 2)) - 6*v_z/T + g
        ACG_x = -12*r_x/(pow(T, 2)) - 6*v_x/T

        ACG_mag = math.sqrt(pow(ACG_z, 2) + pow(ACG_x, 2))
        ACG_angle = -math.atan2(ACG_z, ACG_x) + math.pi/2

        F = m * ACG_mag

        Kp = self.I_y
        Kd = 2 * self.I_y
        err = ACG_angle - beta
        M_y = Kp * err - Kd * dbeta

            #   P velocity controller with set point at v_z = -20
            # Kp = -m
            # F = Kp * (v_z + 20) + m * g
        # else:
            #   After low gate
            #   PD altitude controller with set point at r_z = 0
            #   Damping ratio is 2
            # Kp = -m * 0.25
            # Kd = -2 * 1 * m
            # F = Kp * r_z + Kd * v_z + m * g
            # M_y = 0

        #   Clamp F
        if F < 0:       F = 0
        if F > self.F_max:   F = self.F_max
        if m <= 300:    F = 0

        #   Clamp M_y
        if M_y > self.M_y_max: M_y = self.M_y_max
        if M_y < -self.M_y_max: M_y = -self.M_y_max
        if m <= 300:    F = 0

        # F = 1000

        dxdt = [
            -F/(self.I_spt * g_0) - M_y/(self.d_y * self.I_spm * g_0),
            v_x,
            v_z,
            F*np.sin(beta)/m,
            F*np.cos(beta)/m - g,
            dbeta,
            M_y/self.I_y 
        ]
        return dxdt