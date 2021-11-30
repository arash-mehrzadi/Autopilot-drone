#!/usr/bin/python

import quad_helper
import numpy as np
import time
import math
import sim

floor_x = 5
floor_y = 5
simulation_time = 10 #sec

def calc_pos(pos):
    if(pos[0]<=floor_x):
        pos = pos + np.array([0.1, 0, 0]) 
    
    if(pos[1]<=floor_y ):
        pos = pos + np.array([0, 0.1, 0]) 
    
    #pos = pos + np.array([0.1, 0.1, 0])
    return pos

quad_functions = None
try:
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID == -1:
        print("Failed to connect to remote API Server")
        quad_functions.stop_sim()
        sim.simxFinish(clientID)

    quad_functions = quad_helper.quad_helper(clientID)

    print('Main Script Started')
    quad_functions.init_sensors()
    quad_functions.start_sim()

    # Setting initial time
    init_time = time.time()
    d1 = 0
   
    # Getting object position with respect to first joint
    #pos = quad_functions.get_position()

    # Getting object orientation with respect to first joint
    orien = quad_functions.get_orientation()
    quad_pos = np.array([0,0,0.1])

    while sim.simxGetConnectionId(clientID) != -1:
        elapsed_time = time.time()-init_time
               
        # Moving to grasping position                
        quad_pos = calc_pos(quad_pos)
        print(quad_pos)
        quad_functions.move_quad(
            [quad_pos[0], quad_pos[1], quad_pos[2]])
                
        #time.sleep(0.3)
        
        sim.simxSynchronousTrigger(clientID)
        print(elapsed_time)
        if elapsed_time>simulation_time:
            raise KeyboardInterrupt

        
except KeyboardInterrupt:

    quad_functions.stop_sim()
    sim.simxFinish(clientID)
