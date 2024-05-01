from simulator import *

def main():

    totalSuccessful = 0
    softLandingErrors = []
    softLandingFuel = []

    for test in range(NUM_TESTS):

        if ((test + 1) % 10 == 0):
            print("Test " + str(test+1) + "/" + str(NUM_TESTS))

        lander = ConvLander(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 10.0)
        successfulLanding, rangeError, fuelConsumed, solution = lander.simulate()

        if (successfulLanding): 
            totalSuccessful += 1
            softLandingErrors.append(rangeError)
            softLandingFuel.append(fuelConsumed)

    print()
    print("Success rate: " + str(round(float(totalSuccessful * 100)/NUM_TESTS, 1)) + "%")

    avgError = sum(softLandingErrors)/totalSuccessful
    avgFuel = sum(softLandingFuel)/totalSuccessful

    print("Avg. range error: " + str(round(avgError, 2)) + " [m]")
    print("Avg. fuel consumption: " + str(round(avgFuel, 2)), " [kg]")
    
    # plotResults(solution)

if (__name__ == "__main__"):
    main()

