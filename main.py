from simulator import *

def main():

    totalSuccessful = 0
    softLandingErrors = []
    softLandingFuel = []

    # print()
    # for test in range(11):
    #     if test == 5: continue
    #     dbeta = -10.0+test*2.0
    # print("deta: " + str(round(dbeta, 1)))
    # lander.simulate()

    # convLander = ConvLander(1.0, 1.0, 1.0, 1.0, 1.0, 0,.0, 0.0)
    # successfulLanding, rangeError, fuelConsumed, convSolution = convLander.simulate()

    tvcLander = TVCLander(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    tvcSolution, r_x, r_z = tvcLander.simulate()

    # tvcTime = (len(tvcSolution)-1)*0.1
    # convTime = (len(convSolution)-1)*0.01
    # syncTimeMax = max(tvcTime, convTime)

    # syncTime = np.linspace(0, syncTimeMax, int(syncTimeMax*10.0)+1)

    # syncTVCX = []
    # syncTVCZ = []
    # syncConvX = []
    # syncConvZ = []
    # syncRX = []
    # syncRZ = []
    # for i in range(len(syncTime)):
    #     if (len(convSolution) > i*10):
    #         syncConvX.append(convSolution[i*10][1])
    #         syncConvZ.append(convSolution[i*10][2])
    #     else:
    #         syncConvX.append(syncConvX[-1])
    #         syncConvZ.append(syncConvZ[-1])

    #     if (len(tvcSolution) > i):
    #         syncTVCX.append(tvcSolution[i][1])
    #         syncTVCZ.append(tvcSolution[i][2])
    #     else:
    #         syncTVCX.append(syncTVCX[-1])
    #         syncTVCZ.append(syncTVCZ[-1])

    #     if (len(r_x) > i):
    #         syncRX.append(r_x[i])
    #         syncRZ.append(r_z[i])
    #     else:
    #         syncRX.append(syncRX[-1])
    #         syncRZ.append(syncRZ[-1])

    # plt.figure()
    # plt.plot(syncTVCX, syncTVCZ, label='TVC')
    # plt.plot(syncConvX, syncConvZ, label='Conv')
    # plt.plot(syncRX, syncRZ, label='Guidance')
    # plt.legend(loc='best')
    # plt.xlabel('x [m]')
    # plt.ylabel('z [m]')

    # plt.show()
    


    #     if (successfulLanding): 
    #         totalSuccessful += 1
    #         softLandingErrors.append(rangeError)
    #         softLandingFuel.append(fuelConsumed)

    # print()
    # print("Success rate: " + str(round(float(totalSuccessful * 100)/NUM_TESTS, 1)) + "%")

    # print("Soft landing: " + str(successfulLanding))
    # print("Range error: " + str(round(rangeError, 2)) + " [m]")
    # print("Fuel consumption: " + str(round(fuelConsumed, 2)), " [kg]")

    # avgError = sum(softLandingErrors)/totalSuccessful
    # avgFuel = sum(softLandingFuel)/totalSuccessful

    # print("Avg. range error: " + str(round(avgError, 2)) + " [m]")
    # print("Avg. fuel consumption: " + str(round(avgFuel, 2)), " [kg]")
    
    # plotResults(convSolution)
    # plotTVCResults(solution)

if (__name__ == "__main__"):
    main()

