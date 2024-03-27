from simulator import *

def main():
    lander = ConvLander()
    successfulLanding, solution = lander.simulate()
    print(successfulLanding)
    plotResults(solution)

if (__name__ == "__main__"):
    main()

