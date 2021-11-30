import sim

propellers = ['propeller1Vel', 'propeller2Vel',
              'propeller3Vel', 'propeller4Vel']


def init_rotors(clientID):
    # Clear all signals
    for i in range(len(propellers)):
        sim.simxClearFloatSignal(
            clientID, propellers[i], sim.simx_opmode_oneshot)

    # Set all propellers to zero
    for i in range(len(propellers)):
        sim.simxSetFloatSignal(
            clientID, propellers[i], 0.0, sim.simx_opmode_oneshot)


def move_rotors(clientID, propeller_vels):
    sim.simxSetFloatSignal(
        clientID, propellers[0], propeller_vels[0], sim.simx_opmode_oneshot)
    sim.simxSetFloatSignal(
        clientID, propellers[1], propeller_vels[1], sim.simx_opmode_oneshot)
    sim.simxSetFloatSignal(
        clientID, propellers[2], propeller_vels[2], sim.simx_opmode_oneshot)
    sim.simxSetFloatSignal(
        clientID, propellers[3], propeller_vels[3], sim.simx_opmode_oneshot)
