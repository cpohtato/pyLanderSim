from simulator import *

def main():

    totalSuccessful = 0
    softLandingErrors = []
    softLandingFuel = []

    print()
    for test in range(11):
        if test == 5: continue
        dbeta = -10.0+test*2.0
        lander = TVCLander(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, dbeta, 0.0, 0.0)
        # successfulLanding, rangeError, fuelConsumed, solution = lander.simulate()
        print("deta: " + str(round(dbeta, 1)))
        lander.simulate()

    #     if (successfulLanding): 
    #         totalSuccessful += 1
    #         softLandingErrors.append(rangeError)
    #         softLandingFuel.append(fuelConsumed)

    # print()
    # print("Success rate: " + str(round(float(totalSuccessful * 100)/NUM_TESTS, 1)) + "%")

    # print("Range error: " + str(round(rangeError, 2)) + " [m]")
    # print("Fuel consumption: " + str(round(fuelConsumed, 2)), " [kg]")

    # avgError = sum(softLandingErrors)/totalSuccessful
    # avgFuel = sum(softLandingFuel)/totalSuccessful

    # print("Avg. range error: " + str(round(avgError, 2)) + " [m]")
    # print("Avg. fuel consumption: " + str(round(avgFuel, 2)), " [kg]")
    
    # plotResults(solution)
    # plotTVCResults(solution)

if (__name__ == "__main__"):
    main()

