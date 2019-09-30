#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint angles."""
import time
import numpy as np

import py_finger
import py_real_finger

#import ipdb

finger_data = py_finger.Data()
finger_backend = py_real_finger.create_random_finger_backend(finger_data)
finger = py_finger.Frontend(finger_data)
#finger_backend.calibrate() - this is for when using the RealFingerDriver

desired_torque = np.zeros(3)

f = open("log.csv", "a+")
finger_logger = py_real_finger.Logger(finger_data)
finger_logger.run()

while True:
    for _ in range(1000):
        t = finger.append_desired_action(desired_torque)
        #breakpoint()
        pos = finger.get_observation(t).angle

        #WHAT TIME IS THIS GIVING?
        #start_time = time.time()
        #finger_logger.write() #is this correct usage- calling write at every time instant? #no it's not
        #print("--- %s seconds ---" %(time.time() - start_time))

        #f.write("%6.3f %6.3f %6.3f \n" % (pos[0], pos[1], pos[2]))

        #("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
