from simulator import *

def main():
    numSteps = round(SIM_LENGTH/DT)+1
    t = np.linspace(0, SIM_LENGTH, numSteps)

    lander = ConvLander()

    sol = odeint(lander.stateTransition3DoF, lander.getIC(), t)
    plotResults(sol, t)

if (__name__ == "__main__"):
    main()

