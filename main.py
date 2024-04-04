from simulator import *

def main():
    lander = ConvLander()
    solution = lander.simulate()
    plotResults(solution)

if (__name__ == "__main__"):
    main()

